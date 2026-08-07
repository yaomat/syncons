`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * proposal_buffer v1
 *
 * Fixed-size slot buffer for proposal payloads.
 *
 * - DMA engine writes proposal payloads into the buffer through
 *   the Corundum DMA RAM write interface.
 * - proposal_dma_reader uses the current tail slot before issuing
 *   a DMA read command to fill the next slot.
 *
 * v1 assumptions:
 * - one proposal = one fixed-size slot
 * - DMA read len = PROPOSAL_SLOT_BYTES
 * - TX read len = PROPOSAL_SLOT_BYTES
 * - single outstanding DMA read in proposal_dma_reader
 */

module proposal_buffer #
(
    parameter DMA_LEN_WIDTH = 16,
    
    parameter RAM_SEL_WIDTH = 1,
    parameter RAM_SEL_PROP = 0,

    parameter RAM_ADDR_WIDTH = 16,
    parameter RAM_SEG_COUNT = 2,
    parameter RAM_SEG_DATA_WIDTH = 256*2/RAM_SEG_COUNT,
    parameter RAM_SEG_BE_WIDTH = RAM_SEG_DATA_WIDTH/8,
    parameter RAM_SEG_ADDR_WIDTH = RAM_ADDR_WIDTH-$clog2(RAM_SEG_COUNT*RAM_SEG_BE_WIDTH),
    parameter RAM_PIPELINE = 2,

    parameter PROPOSAL_SLOT_BYTES = 1024,
    parameter PROPOSAL_SLOT_COUNT = 64
)
(
    input  wire                     clk,
    input  wire                     rst,

    // Tail slot interface to proposal_dma_reader
    output wire                                             tail_slot_valid,
    output wire [RAM_ADDR_WIDTH-1:0]                        tail_slot_addr,
    output wire [DMA_LEN_WIDTH-1:0]                         tail_slot_len,

    // commit current tail slot
    input  wire                                             tail_commit_valid,
    output wire                                             tail_commit_ready,

    // DMA RAM write interface
    input  wire [RAM_SEG_COUNT*RAM_SEL_WIDTH-1:0]           dma_ram_wr_cmd_sel,
    input  wire [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0]        dma_ram_wr_cmd_be,
    input  wire [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]      dma_ram_wr_cmd_data,
    input  wire [RAM_SEG_COUNT*RAM_SEG_ADDR_WIDTH-1:0]      dma_ram_wr_cmd_addr,
    input  wire [RAM_SEG_COUNT-1:0]                         dma_ram_wr_cmd_valid,
    output wire [RAM_SEG_COUNT-1:0]                         dma_ram_wr_cmd_ready,
    output wire [RAM_SEG_COUNT-1:0]                         dma_ram_wr_done,

    // TX streaming interface to tx_engine
    // proposal_buffer streams the current head slot to tx_engine
    output wire [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]      buf_rd_data,
    output wire [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0]        buf_rd_be,
    output wire                                             buf_rd_valid,    
    input  wire                                             buf_rd_ready,
    output wire                                             buf_tx_last,
    output wire [DMA_LEN_WIDTH-1:0]                         buf_tx_len,
    output wire                                             buf_empty
);

localparam [RAM_SEL_WIDTH-1:0] RAM_SEL_PROP_VALUE = RAM_SEL_PROP;

localparam integer PROPOSAL_SLOT_BYTE_ADDR_WIDTH = $clog2(PROPOSAL_SLOT_BYTES);
localparam integer PROPOSAL_SLOT_BEAT_COUNT = PROPOSAL_SLOT_BYTES / (RAM_SEG_COUNT * RAM_SEG_BE_WIDTH);
localparam integer BUFFER_RAM_SIZE = PROPOSAL_SLOT_BYTES * PROPOSAL_SLOT_COUNT;

localparam integer SLOT_PTR_WIDTH = PROPOSAL_SLOT_COUNT > 1 ? $clog2(PROPOSAL_SLOT_COUNT) : 1;
localparam integer SLOT_COUNT_WIDTH = $clog2(PROPOSAL_SLOT_COUNT+1);

localparam [DMA_LEN_WIDTH-1:0] PROPOSAL_SLOT_BYTES_LEN = PROPOSAL_SLOT_BYTES;
localparam [SLOT_COUNT_WIDTH-1:0] PROPOSAL_SLOT_COUNT_VALUE = PROPOSAL_SLOT_COUNT;

localparam integer TX_BEAT_INDEX_WIDTH = PROPOSAL_SLOT_BEAT_COUNT > 1 ? $clog2(PROPOSAL_SLOT_BEAT_COUNT) : 1;
localparam [TX_BEAT_INDEX_WIDTH-1:0] TX_LAST_BEAT_INDEX = PROPOSAL_SLOT_BEAT_COUNT - 1;

localparam [63:0] BUFFER_RAM_SIZE_64 = BUFFER_RAM_SIZE;
localparam [63:0] PROPOSAL_SLOT_BYTES_64 = PROPOSAL_SLOT_BYTES;
localparam [63:0] RAM_ADDR_LIMIT_64 = 64'd1 << RAM_ADDR_WIDTH;
localparam [63:0] DMA_LEN_LIMIT_64 = 64'd1 << DMA_LEN_WIDTH;

initial begin
    if (PROPOSAL_SLOT_BYTES == 0) begin
        $error("PROPOSAL_SLOT_BYTES must be greater than 0");
        $finish;
    end

    if (PROPOSAL_SLOT_BYTES & (PROPOSAL_SLOT_BYTES-1)) begin
        $error("PROPOSAL_SLOT_BYTES (%0d) must be a power of 2", PROPOSAL_SLOT_BYTES);
        $finish;
    end

    if (PROPOSAL_SLOT_BYTES % (RAM_SEG_COUNT * RAM_SEG_BE_WIDTH) != 0) begin
        $error("PROPOSAL_SLOT_BYTES (%0d) must be a multiple of RAM_SEG_COUNT * RAM_SEG_BE_WIDTH (%0d)", PROPOSAL_SLOT_BYTES, RAM_SEG_COUNT * RAM_SEG_BE_WIDTH);
        $finish;
    end

    if (PROPOSAL_SLOT_COUNT == 0) begin
        $error("PROPOSAL_SLOT_COUNT must be greater than 0");
        $finish;
    end

    if (PROPOSAL_SLOT_COUNT & (PROPOSAL_SLOT_COUNT-1)) begin
        $error("PROPOSAL_SLOT_COUNT (%0d) must be a power of 2", PROPOSAL_SLOT_COUNT);
        $finish;
    end

    if (BUFFER_RAM_SIZE_64 > RAM_ADDR_LIMIT_64) begin
        $error("BUFFER_RAM_SIZE (%0d) exceeds RAM_ADDR_WIDTH (%0d)", BUFFER_RAM_SIZE, RAM_ADDR_WIDTH);
        $finish;
    end

    if (PROPOSAL_SLOT_BYTES_64 > DMA_LEN_LIMIT_64) begin
        $error("PROPOSAL_SLOT_BYTES (%0d) exceeds DMA_LEN_LIMIT (%0d)", PROPOSAL_SLOT_BYTES, DMA_LEN_LIMIT_64);
        $finish;
    end

    if (SLOT_PTR_WIDTH > RAM_ADDR_WIDTH) begin
        $error("SLOT_PTR_WIDTH (%0d) exceeds RAM_ADDR_WIDTH (%0d)", SLOT_PTR_WIDTH, RAM_ADDR_WIDTH);
        $finish;
    end
end

// ==============================================================================
//      Internal Registers and Wires
// ==============================================================================
reg [SLOT_PTR_WIDTH-1:0]    head_ptr_reg = {SLOT_PTR_WIDTH{1'b0}}, head_ptr_next;
reg [SLOT_PTR_WIDTH-1:0]    tail_ptr_reg = {SLOT_PTR_WIDTH{1'b0}}, tail_ptr_next;
reg [SLOT_COUNT_WIDTH-1:0]  slot_count_reg = {SLOT_COUNT_WIDTH{1'b0}}, slot_count_next;

wire buffer_empty;
wire buffer_full;

wire tail_commit_fire;
wire head_pop_fire;

localparam [1:0]
    TX_STATE_IDLE       = 2'd0,
    TX_STATE_READ_CMD   = 2'd1,
    TX_STATE_READ_RESP  = 2'd2,
    TX_STATE_OUTPUT     = 2'd3;

reg [1:0]                                   tx_state_reg = TX_STATE_IDLE, tx_state_next;
reg [TX_BEAT_INDEX_WIDTH-1:0]               tx_beat_index_reg = {TX_BEAT_INDEX_WIDTH{1'b0}}, tx_beat_index_next;
reg [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]  buf_rd_data_reg = {RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH{1'b0}}, buf_rd_data_next;
reg [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0]    buf_rd_be_reg = {RAM_SEG_COUNT*RAM_SEG_BE_WIDTH{1'b0}}, buf_rd_be_next;
reg buf_rd_valid_reg = 1'b0, buf_rd_valid_next;
reg buf_tx_last_reg = 1'b0, buf_tx_last_next;

wire [RAM_SEG_ADDR_WIDTH-1:0]  tx_head_slot_ram_addr;
wire [RAM_SEG_ADDR_WIDTH-1:0]  tx_current_rd_addr;
wire [RAM_SEG_ADDR_WIDTH-1:0]  tx_next_rd_addr;

reg [RAM_SEG_ADDR_WIDTH-1:0]    ram_rd_cmd_addr_reg = {RAM_SEG_ADDR_WIDTH{1'b0}}, ram_rd_cmd_addr_next;
reg [RAM_SEG_COUNT-1:0]         ram_rd_cmd_valid_reg = {RAM_SEG_COUNT{1'b0}}, ram_rd_cmd_valid_next;

// internal RAM read interface signals
wire [RAM_SEG_COUNT*RAM_SEG_ADDR_WIDTH-1:0]             ram_rd_cmd_addr;
wire [RAM_SEG_COUNT-1:0]                                ram_rd_cmd_valid;
wire [RAM_SEG_COUNT-1:0]                                ram_rd_cmd_ready;
wire [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]             ram_rd_resp_data;
wire [RAM_SEG_COUNT-1:0]                                ram_rd_resp_valid;
wire [RAM_SEG_COUNT-1:0]                                ram_rd_resp_ready;

wire ram_rd_cmd_ready_all;
wire ram_rd_resp_valid_all;

wire ram_rd_cmd_fire;
wire ram_rd_resp_fire;

// ==============================================================================
// Slot events
// ==============================================================================
assign buffer_empty = slot_count_reg == {SLOT_COUNT_WIDTH{1'b0}};
assign buffer_full = slot_count_reg == PROPOSAL_SLOT_COUNT_VALUE;
assign buf_empty = buffer_empty;

assign tx_head_slot_ram_addr    = head_ptr_reg * PROPOSAL_SLOT_BEAT_COUNT;
assign tx_current_rd_addr       = tx_head_slot_ram_addr + tx_beat_index_reg;
assign tx_next_rd_addr          = tx_head_slot_ram_addr + tx_beat_index_reg + 1'b1;

assign tail_commit_fire = tail_commit_valid && tail_commit_ready;
assign head_pop_fire = buf_rd_valid && buf_rd_ready && buf_tx_last;

assign tail_slot_valid = !buffer_full;
assign tail_slot_addr = ({{(RAM_ADDR_WIDTH - SLOT_PTR_WIDTH){1'b0}}, tail_ptr_reg} << PROPOSAL_SLOT_BYTE_ADDR_WIDTH);
assign tail_slot_len = PROPOSAL_SLOT_BYTES_LEN;
assign tail_commit_ready = !buffer_full;

assign buf_rd_data  = buf_rd_data_reg;
assign buf_rd_be    = buf_rd_be_reg;
assign buf_rd_valid = buf_rd_valid_reg;
assign buf_tx_last  = buf_tx_last_reg;
assign buf_tx_len   = PROPOSAL_SLOT_BYTES_LEN;

assign ram_rd_cmd_ready_all = &ram_rd_cmd_ready;
assign ram_rd_resp_valid_all = &ram_rd_resp_valid;

assign ram_rd_cmd_addr = {RAM_SEG_COUNT{ram_rd_cmd_addr_reg}};
assign ram_rd_cmd_valid = ram_rd_cmd_valid_reg;

assign ram_rd_resp_ready = {RAM_SEG_COUNT{tx_state_reg == TX_STATE_READ_RESP && ram_rd_resp_valid_all}};

assign ram_rd_cmd_fire = (|ram_rd_cmd_valid_reg) && ram_rd_cmd_ready_all;
assign ram_rd_resp_fire = ram_rd_resp_valid_all && (tx_state_reg == TX_STATE_READ_RESP);

// ==============================================================================
//          Combinational logic for next-state and outputs
// ==============================================================================

// ---------------------------------------------------------
//            DMA reader write engine next-state logic 
// ---------------------------------------------------------
always @* begin
    head_ptr_next = head_ptr_reg;
    tail_ptr_next = tail_ptr_reg;
    slot_count_next = slot_count_reg;

    if (tail_commit_fire) begin
        tail_ptr_next = tail_ptr_reg + 1'b1;
    end

    if (head_pop_fire) begin
        head_ptr_next = head_ptr_reg + 1'b1;
    end

    case ({tail_commit_fire, head_pop_fire})
        2'b00: slot_count_next = slot_count_reg;
        2'b01: slot_count_next = slot_count_reg - 1'b1;
        2'b10: slot_count_next = slot_count_reg + 1'b1;
        2'b11: slot_count_next = slot_count_reg;
    endcase
end

// ---------------------------------------------------------
//             TX read engine next-state logic
// ---------------------------------------------------------
// IDLE:
//   - If buffer is not empty, issue a read command to RAM 
//     for the 0th beat of the current head slot.
//
// READ_CMD:
//   - Wait for RAM to accept the read command.
//
// READ_RESP:
//   - Wait for RAM to return the read data.
//
// OUTPUT:
//   - Wait for tx_engine to accept the read data.
//   - If tx_engine accepted the last beat of the
//     current head slot, go back to IDLE. 
//     Otherwise, issue a read command to RAM for 
//     the next beat of the current head slot.
//
always @* begin
    tx_state_next = tx_state_reg;

    tx_beat_index_next = tx_beat_index_reg;

    buf_rd_data_next = buf_rd_data_reg;
    buf_rd_be_next = buf_rd_be_reg;
    buf_rd_valid_next = buf_rd_valid_reg;
    buf_tx_last_next = buf_tx_last_reg;

    ram_rd_cmd_addr_next = ram_rd_cmd_addr_reg;
    ram_rd_cmd_valid_next = ram_rd_cmd_valid_reg;

    case (tx_state_reg)
        TX_STATE_IDLE: begin
            tx_beat_index_next = {TX_BEAT_INDEX_WIDTH{1'b0}};

            buf_rd_valid_next = 1'b0;
            buf_tx_last_next = 1'b0;

            if (!buffer_empty) begin
                ram_rd_cmd_addr_next = tx_current_rd_addr;
                ram_rd_cmd_valid_next = {RAM_SEG_COUNT{1'b1}};
                tx_state_next = TX_STATE_READ_CMD;
            end
        end

        TX_STATE_READ_CMD: begin
            ram_rd_cmd_valid_next = ram_rd_cmd_valid_reg;

            if (ram_rd_cmd_fire) begin
                ram_rd_cmd_valid_next = {RAM_SEG_COUNT{1'b0}};
                tx_state_next = TX_STATE_READ_RESP;
            end
        end

        TX_STATE_READ_RESP: begin
            if (ram_rd_resp_fire) begin
                buf_rd_data_next = ram_rd_resp_data;
                buf_rd_be_next = {RAM_SEG_COUNT*RAM_SEG_BE_WIDTH{1'b1}};
                buf_rd_valid_next = 1'b1;
                buf_tx_last_next = (tx_beat_index_reg == TX_LAST_BEAT_INDEX);

                tx_state_next = TX_STATE_OUTPUT;
            end
        end

        TX_STATE_OUTPUT: begin
            if (buf_rd_valid_reg && buf_rd_ready) begin
                buf_rd_valid_next = 1'b0;

                if (buf_tx_last_reg) begin
                    tx_beat_index_next = {TX_BEAT_INDEX_WIDTH{1'b0}};
                    tx_state_next = TX_STATE_IDLE;
                end else begin
                    tx_beat_index_next = tx_beat_index_reg + 1'b1;

                    ram_rd_cmd_addr_next = tx_next_rd_addr;
                    ram_rd_cmd_valid_next = {RAM_SEG_COUNT{1'b1}};

                    tx_state_next = TX_STATE_READ_CMD;
                end
            end
        end

        default: begin
            tx_state_next = TX_STATE_IDLE;
        end
    endcase
end

// ==============================================================================
//              Sequential logic for registers
// ==============================================================================
always @(posedge clk) begin
    head_ptr_reg <= head_ptr_next;
    tail_ptr_reg <= tail_ptr_next;
    slot_count_reg <= slot_count_next;

    tx_state_reg <= tx_state_next;
    tx_beat_index_reg <= tx_beat_index_next;

    buf_rd_data_reg <= buf_rd_data_next;
    buf_rd_be_reg <= buf_rd_be_next;
    buf_rd_valid_reg <= buf_rd_valid_next;
    buf_tx_last_reg <= buf_tx_last_next;

    ram_rd_cmd_addr_reg <= ram_rd_cmd_addr_next;
    ram_rd_cmd_valid_reg <= ram_rd_cmd_valid_next;

    if (rst) begin
        head_ptr_reg <= {SLOT_PTR_WIDTH{1'b0}};
        tail_ptr_reg <= {SLOT_PTR_WIDTH{1'b0}};
        slot_count_reg <= {SLOT_COUNT_WIDTH{1'b0}};

        tx_state_reg <= TX_STATE_IDLE;
        tx_beat_index_reg <= {TX_BEAT_INDEX_WIDTH{1'b0}};

        buf_rd_data_reg <= {RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH{1'b0}};
        buf_rd_be_reg <= {RAM_SEG_COUNT*RAM_SEG_BE_WIDTH{1'b0}};
        buf_rd_valid_reg <= 1'b0;
        buf_tx_last_reg <= 1'b0;

        ram_rd_cmd_addr_reg <= {RAM_SEG_ADDR_WIDTH{1'b0}};
        ram_rd_cmd_valid_reg <= {RAM_SEG_COUNT{1'b0}};
    end
end


// ==============================================================================
//              Proposal payload RAM
// ==============================================================================

dma_psdpram #(
    .SIZE(BUFFER_RAM_SIZE),
    .SEG_COUNT(RAM_SEG_COUNT),
    .SEG_DATA_WIDTH(RAM_SEG_DATA_WIDTH),
    .SEG_BE_WIDTH(RAM_SEG_BE_WIDTH),
    .SEG_ADDR_WIDTH(RAM_SEG_ADDR_WIDTH),
    .PIPELINE(RAM_PIPELINE)
)
proposal_ram_inst (
    .clk(clk),
    .rst(rst),

    // Write interface
    .wr_cmd_be(dma_ram_wr_cmd_be),
    .wr_cmd_addr(dma_ram_wr_cmd_addr),
    .wr_cmd_data(dma_ram_wr_cmd_data),
    .wr_cmd_valid(dma_ram_wr_cmd_valid),
    .wr_cmd_ready(dma_ram_wr_cmd_ready),
    .wr_done(dma_ram_wr_done),

    // Read interface
    .rd_cmd_addr(ram_rd_cmd_addr),
    .rd_cmd_valid(ram_rd_cmd_valid),
    .rd_cmd_ready(ram_rd_cmd_ready),
    .rd_resp_data(ram_rd_resp_data),
    .rd_resp_valid(ram_rd_resp_valid),
    .rd_resp_ready(ram_rd_resp_ready)
);

endmodule
