`timescale 1ns / 1ps

module consensus_tx #(
    parameter integer P_DATA_WIDTH = 512,
    parameter integer P_KEEP_WIDTH = P_DATA_WIDTH / 8,
    parameter integer P_ID_WIDTH = 12,
    parameter integer P_DEST_WIDTH = 4,
    parameter integer P_NODE_ID = 0,
    parameter integer P_NODE_COUNT = 3,
    parameter integer P_LOG_ITEM_LEN = 32, // bytes
    parameter [47:0] P_SRC_MAC = 48'h02_00_00_00_00_00,
    parameter [15:0] P_ETHERNET_TYPE = 16'h88B5,
    
    parameter DMA_LEN_WIDTH = 16,

    parameter RAM_SEG_COUNT = 2,
    parameter RAM_SEG_DATA_WIDTH = 256*2/RAM_SEG_COUNT,
    parameter RAM_SEG_BE_WIDTH = RAM_SEG_DATA_WIDTH/8,

    parameter PROPOSAL_SLOT_BYTES = 1024
) (
    // clock and reset
    input wire                              clk,
    input wire                              rst,

    input wire [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]   buf_rd_data,
    input wire [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0]     buf_rd_be,
    input wire                                          buf_rd_valid,
    output reg                                          buf_rd_ready,
    input wire                                          buf_tx_last,
    input wire [DMA_LEN_WIDTH-1:0]                      buf_tx_len,
    input wire                                          buf_empty, 

    // Control and Data
    input wire                              i_tx_allowed,
    input wire [63:0]                       i_current_round_id,
    input wire [63:0]                       i_current_run_id,
    input wire [P_NODE_COUNT-1:0]           i_knowledge_vec,
    input wire                              i_halt,

    // AXI Stream Master Output
    output reg [P_DATA_WIDTH-1:0]           m_axis_tdata,
    output reg [P_KEEP_WIDTH-1:0]           m_axis_tkeep,
    output reg                              m_axis_tvalid,
    output reg                              m_axis_tlast,
    output reg                              m_axis_tuser,
    output reg [P_ID_WIDTH-1:0]             m_axis_tid,
    output reg [P_DEST_WIDTH-1:0]           m_axis_tdest,
    input wire                              m_axis_tready,

    // Status outputs
    output wire [31:0]                                      tx_slot_count,
    output wire [31:0]                                      tx_beat_count,
    output wire [31:0]                                      tx_error_count
);
//------------------------------------------------
//         Endianess Conversion
//------------------------------------------------
// helper function for byte swapping
function [15:0] to_big_endian_16(input [15:0] in);
    to_big_endian_16 = {in[7:0], in[15:8]};
endfunction

function [63:0] to_big_endian_64(input [63:0] in);
    to_big_endian_64 = {in[7:0], in[15:8], in[23:16], in[31:24],
                       in[39:32], in[47:40], in[55:48], in[63:56]};
endfunction

//------------------------------------------------
//           parameter Definitions
//------------------------------------------------
localparam S_IDLE                 = 3'b00;
localparam S_BROADCAST_START      = 3'b01;
localparam S_BROADCAST_SEND       = 3'b10;
localparam S_BROADCAST_WAIT       = 3'b11;
localparam S_HALT                 = 3'b100;

reg [2:0] state;
reg [7:0]   r_target_node_id;

reg [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0] current_payload;

reg last_tx_allowed;
wire tx_allowed_pulse = i_tx_allowed && !last_tx_allowed; // Detect rising edge of tx_allowed

// MAC address
reg [47:0]  v_dest_mac;

always @(*) begin
    // Default value
    v_dest_mac = 48'hFF_FF_FF_FF_FF_FF; // Broadcast MAC

    // Select destination MAC based on destination node ID
    case (r_target_node_id)
        0: v_dest_mac = 48'h00_0a_35_06_50_94;
        1: v_dest_mac = 48'h00_0a_35_06_09_24;
        2: v_dest_mac = 48'h00_0a_35_06_0b_84;
        3: v_dest_mac = 48'h00_0a_35_06_09_3c;
        4: v_dest_mac = 48'h00_0a_35_06_0b_72;
        default: v_dest_mac = 48'hFF_FF_FF_FF_FF_FF; // Broadcast MAC
    endcase
end

//------------------------------------------------
//         Packet Construction (Single Cycle)
//------------------------------------------------
// Construct packet flit
// Packet format:
// [ Ethernet Header ]
//   - Destination MAC (48 bits)
//   - Source MAC (48 bits)
//   - Ethertype (16 bits)
// [ Consensus Header ]
//  - Slot ID (64 bits)
//  - Knowledge Vector (8 bits)
//  - Node ID (8 bits)
//  - Payload (32 bytes)

reg [P_DATA_WIDTH-1:0]      v_packet_flit;
always @(*) begin
    v_packet_flit = {P_DATA_WIDTH{1'b0}};

    // ------- Ethernet Header -------
    v_packet_flit[7:0]       = v_dest_mac[47:40];
    v_packet_flit[15:8]       = v_dest_mac[39:32];
    v_packet_flit[23:16]      = v_dest_mac[31:24];
    v_packet_flit[31:24]      = v_dest_mac[23:16];
    v_packet_flit[39:32]      = v_dest_mac[15:8];
    v_packet_flit[47:40]      = v_dest_mac[7:0];

    v_packet_flit[55:48]      = P_SRC_MAC[47:40];
    v_packet_flit[63:56]      = P_SRC_MAC[39:32];
    v_packet_flit[71:64]      = P_SRC_MAC[31:24];
    v_packet_flit[79:72]      = P_SRC_MAC[23:16];
    v_packet_flit[87:80]     = P_SRC_MAC[15:8];
    v_packet_flit[95:88]     = P_SRC_MAC[7:0];

    v_packet_flit[12*8 +: 16] = to_big_endian_16(P_ETHERNET_TYPE);

    // ------- Consensus Header -------
    v_packet_flit[14*8 +: 64]  = to_big_endian_64(i_current_run_id);
    v_packet_flit[22*8 +: P_NODE_COUNT]   = i_knowledge_vec;
    v_packet_flit[23*8 +: 8]   = P_NODE_ID[7:0];
    v_packet_flit[24*8 +: 64]  = to_big_endian_64(i_current_round_id);

    // ------- Payload -------
    v_packet_flit[32*8 +: P_LOG_ITEM_LEN*8] = 
    {
        to_big_endian_64(current_payload[63:0]),
        to_big_endian_64(current_payload[127:64]),
        to_big_endian_64(current_payload[191:128]),
        to_big_endian_64(current_payload[255:192])
    }; // not sure if this is right, but doing this to be consistent with rx side parsing
end

// Debug Parameters

localparam integer RAM_BEAT_BYTES                   = RAM_SEG_COUNT * RAM_SEG_BE_WIDTH;
localparam integer PROPOSAL_SLOT_BEAT_COUNT         = PROPOSAL_SLOT_BYTES / RAM_BEAT_BYTES;
localparam integer BEAT_INDEX_WIDTH                 = PROPOSAL_SLOT_BEAT_COUNT > 1 ? $clog2(PROPOSAL_SLOT_BEAT_COUNT) : 1;
localparam [BEAT_INDEX_WIDTH-1:0] LAST_BEAT_INDEX   = PROPOSAL_SLOT_BEAT_COUNT - 1;
localparam [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0] FULL_BE = {RAM_SEG_COUNT*RAM_SEG_BE_WIDTH{1'b1}};

reg [31:0] tx_slot_count_reg = 32'd0;
reg [31:0] tx_beat_count_reg = 32'd0;
reg [31:0] tx_error_count_reg = 32'd0;

reg [BEAT_INDEX_WIDTH-1:0] beat_index_reg = {BEAT_INDEX_WIDTH{1'b0}};

assign tx_slot_count  = tx_slot_count_reg;
assign tx_beat_count  = tx_beat_count_reg;
assign tx_error_count = tx_error_count_reg;

wire be_error = tx_allowed_pulse && (buf_rd_be != FULL_BE);
wire last_missing_error = tx_allowed_pulse && (beat_index_reg == LAST_BEAT_INDEX) && !buf_tx_last;
wire last_early_error = tx_allowed_pulse && (beat_index_reg != LAST_BEAT_INDEX) && buf_tx_last;
wire [31:0] error_inc = (be_error ? 32'd1 : 32'd0) + (last_missing_error ? 32'd1 : 32'd0) + (last_early_error ? 32'd1 : 32'd0);

function [7:0] count_ones;
    input [P_NODE_COUNT-1:0] vec;
    integer idx;
    begin
        count_ones = 0;
        for (idx = 0; idx < P_NODE_COUNT; idx = idx + 1) begin
            if (vec[idx]) begin
                count_ones = count_ones + 1;
            end
        end
    end
endfunction

wire [7:0] knowledge_count = (count_ones(i_knowledge_vec) - 1);
reg [7:0] nodes_completed_reg;

//------------------------------------------------
//         State Machine
//------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        state <= S_IDLE;
        m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
        m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
        m_axis_tuser <= 1'b0;
        m_axis_tid <= 8'b0;
        m_axis_tdest <= 8'b0;
        r_target_node_id <= 8'b0;
        last_tx_allowed <= 1'b0;
        buf_rd_ready <= 1'b0;
    end else begin
        last_tx_allowed <= i_tx_allowed;

        case (state)
            S_IDLE: begin
                // clear outputs
                m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                m_axis_tvalid <= 1'b0;
                m_axis_tlast <= 1'b0;
                m_axis_tuser <= 1'b0;
                m_axis_tid <= 8'b0; // Use target node ID as TID
                m_axis_tdest <= 8'b0; // Use target node ID as DEST
                nodes_completed_reg <= 8'b0;
                
                r_target_node_id <= 0;
                buf_rd_ready <= 1'b0;
                current_payload <= {RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH{1'b0}};

                if (tx_allowed_pulse && !i_halt) begin
                    // Start broadcasting to all nodes
                    state <= S_BROADCAST_START;
                end
            end

            S_BROADCAST_START: begin
                if (!i_tx_allowed) begin
                    state <= S_IDLE; // Abort if not allowed
                    m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                    m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast <= 1'b0;
                    m_axis_tuser <= 1'b0;
                    m_axis_tid <= 8'b0;
                    m_axis_tdest <= 8'b0;
                    buf_rd_ready <= 1'b0;
                end
                else begin
                    buf_rd_ready <= 1'b1;
                    current_payload <= buf_rd_valid ? buf_rd_data : current_payload;

                    state <= S_BROADCAST_SEND;
                end
            end

            
            S_BROADCAST_SEND: begin
                buf_rd_ready <= 1'b0;
                if (!i_tx_allowed) begin
                    state <= S_IDLE; // Abort if not allowed
                    m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                    m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast <= 1'b0;
                    m_axis_tuser <= 1'b0;
                    m_axis_tid <= 8'b0;
                    m_axis_tdest <= 8'b0;
                    current_payload <= {RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH{1'b0}};
                end else if (m_axis_tready) begin
                    if (r_target_node_id != P_NODE_ID && (i_knowledge_vec[r_target_node_id])) begin
                        m_axis_tdata <= v_packet_flit;
                        m_axis_tkeep <= {P_KEEP_WIDTH{1'b1}}; // All bytes valid
                        m_axis_tvalid <= 1'b1;
                        m_axis_tuser <= P_NODE_ID;
                        m_axis_tid <= P_NODE_ID;
                        m_axis_tdest <= r_target_node_id[3:0];
                        nodes_completed_reg <= nodes_completed_reg + 1;
                        m_axis_tlast <= 1'b1;
                    end else begin
                        m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                        m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                        m_axis_tvalid <= 1'b0;
                        m_axis_tuser <= 1'b0;
                        m_axis_tid <= 8'b0;
                        m_axis_tdest <= 8'b0;
                    end

                    if (r_target_node_id + 1 == P_NODE_COUNT || ((nodes_completed_reg + 1) == knowledge_count && r_target_node_id != P_NODE_ID && (i_knowledge_vec[r_target_node_id]))) begin
                        // Finished broadcasting
                        state <= S_BROADCAST_WAIT;
                        r_target_node_id <= 0;
                    end else if ((r_target_node_id + 1) == P_NODE_ID && P_NODE_ID + 1 < P_NODE_COUNT) begin
                        // Skip self node
                        r_target_node_id <= r_target_node_id + 2;
                        state <= S_BROADCAST_SEND;
                    end else if ((r_target_node_id + 1) == P_NODE_ID && P_NODE_ID + 1 == P_NODE_COUNT) begin
                        // Skip self node and finish broadcasting
                        state <= S_BROADCAST_WAIT;
                        r_target_node_id <= 0;
                    end else begin
                        // Wait for the next cycle to send the next packet
                        r_target_node_id <= r_target_node_id + 1;
                        state <= S_BROADCAST_SEND;
                    end
                end
            end

            S_BROADCAST_WAIT: begin
                if (!i_tx_allowed) begin
                    state <= S_IDLE; // Abort if not allowed
                    m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                    m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast <= 1'b0;
                    m_axis_tuser <= 1'b0;
                    m_axis_tid <= 8'b0;
                    m_axis_tdest <= 8'b0;
                    buf_rd_ready <= 1'b0;
                end else if (buf_rd_valid) begin
                    state <= S_BROADCAST_SEND;
                    current_payload <= buf_rd_data;
                    buf_rd_ready <= 1'b1;
                end else if (buf_empty) begin
                    state <= S_IDLE; // No more data to send
                    m_axis_tdata <= {P_DATA_WIDTH{1'b0}};
                    m_axis_tkeep <= {P_KEEP_WIDTH{1'b0}};
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast <= 1'b0;
                    m_axis_tuser <= 1'b0;
                    m_axis_tid <= 8'b0;
                    m_axis_tdest <= 8'b0;
                    buf_rd_ready <= 1'b0;
                end
            end

            default: state <= S_IDLE;
        endcase
    end 
end

// =====================================================================
//              Debug registers
// =====================================================================

always @(posedge clk) begin
    if (rst) begin
        tx_slot_count_reg  <= 32'd0;
        tx_beat_count_reg  <= 32'd0;
        tx_error_count_reg <= 32'd0;
        beat_index_reg       <= {BEAT_INDEX_WIDTH{1'b0}};
    end else begin
        if (tx_allowed_pulse) begin

            tx_beat_count_reg <= tx_beat_count_reg + 1;

            if (beat_index_reg == LAST_BEAT_INDEX) begin

                tx_slot_count_reg <= tx_slot_count_reg + 1;
                beat_index_reg <= {BEAT_INDEX_WIDTH{1'b0}};

            end else begin

                beat_index_reg <= beat_index_reg + 1;

            end

            tx_error_count_reg <= tx_error_count_reg + error_inc;
        end
    end
end


endmodule
