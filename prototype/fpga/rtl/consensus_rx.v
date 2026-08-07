`timescale 1ns / 1ps

module consensus_rx #(
    parameter P_NODE_COUNT = 3,
    parameter P_NODE_ID = 0,
    parameter P_DATA_WIDTH = 512, // Ethernet frame data width of FPGA
    parameter P_KEEP_WIDTH = P_DATA_WIDTH / 8,
    parameter P_PORTS_PER_IF = 1,
    parameter P_ID_WIDTH = P_PORTS_PER_IF > 1 ? $clog2(P_PORTS_PER_IF) : 1,
    parameter P_DEST_WIDTH = 8,
    parameter P_USER_WIDTH = 1,
    parameter P_ETHERNET_TYPE = 16'h88B5,
    parameter integer   P_LOG_ITEM_LEN  = 32,      // 40 bytes default, smaller to fit room for other fields in the test frame
    
    parameter RAM_SEG_COUNT = 2,
    parameter RAM_SEG_DATA_WIDTH = 256*2/RAM_SEG_COUNT,
    parameter RAM_SEG_BE_WIDTH = RAM_SEG_DATA_WIDTH/8,

    parameter COMMIT_SLOT_BYTES = 1024,
    parameter MAX_COMMITS_PER_ROUND = 10, // need to do actual math to calculate this
    parameter LOG_MAX_COMMITS = $clog2(MAX_COMMITS_PER_ROUND)
) (
    // clock and reset
    input wire                          clk,
    input wire                          rst,

    // Control Signals from Scheduler inside consensus core
    input wire                          i_rx_enabled,
    input wire [63:0]                   i_current_run_id,
    input wire [63:0]                   i_current_round_id,
    input wire                          i_halt,

    // Commit stream output to commit_buffer
    output reg [RAM_SEG_COUNT*RAM_SEG_DATA_WIDTH-1:0]       commit_in_data,
    output reg [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0]         commit_in_be,
    output reg                                              commit_in_valid,
    output reg                                              commit_in_last,
    input  wire                                             commit_in_ready,

    // AXI Stream Slave Input
    input wire [P_DATA_WIDTH-1:0]       s_axis_tdata,
    input wire [P_KEEP_WIDTH-1:0]       s_axis_tkeep,
    input wire                          s_axis_tvalid,
    output wire                         s_axis_tready,
    input wire                          s_axis_tlast,
    input wire [P_ID_WIDTH-1:0]         s_axis_tid,
    input wire [P_DEST_WIDTH-1:0]       s_axis_tdest,
    input wire [P_USER_WIDTH-1:0]       s_axis_tuser,

    // Parsed Output to Consensus Module
    output reg                              o_rx_valid,     // high when a valid packet is parsed
    output reg [7:0]                        o_rx_node_id,   // node ID extracted from packet
    output reg [P_NODE_COUNT-1:0]           o_rx_sound_bitmap, // sound bitmap extracted from packet
    output reg [31:0]                       o_rx_run_id,
    output reg [31:0]                       o_rx_round_id
);

//------------------------------------------------
//         Interface Logic
//------------------------------------------------
// The consensus model must run at the line rate of incoming packets.
// Therefore, we assume that the AXI Stream input is always ready to accept data.
assign s_axis_tready = 1'b1; // Always ready to accept data

localparam integer RAM_BEAT_BYTES =
    RAM_SEG_COUNT * RAM_SEG_BE_WIDTH;

localparam integer COMMIT_SLOT_BEAT_COUNT =
    COMMIT_SLOT_BYTES / RAM_BEAT_BYTES;

localparam integer COMMIT_SLOT_BEAT_INDEX_WIDTH =
    COMMIT_SLOT_BEAT_COUNT > 1 ? $clog2(COMMIT_SLOT_BEAT_COUNT) : 1;

localparam [COMMIT_SLOT_BEAT_INDEX_WIDTH-1:0] COMMIT_LAST_BEAT_INDEX =
    COMMIT_SLOT_BEAT_COUNT - 1;

localparam integer DATA_WORD_COUNT =
    RAM_SEG_COUNT * RAM_SEG_DATA_WIDTH / 32;

localparam [RAM_SEG_COUNT*RAM_SEG_BE_WIDTH-1:0] FULL_BE =
    {RAM_SEG_COUNT*RAM_SEG_BE_WIDTH{1'b1}};

// states
localparam S_IDLE = 2'b00;
localparam S_RECEIVE = 2'b01;
localparam S_COMMIT = 2'b10;
localparam S_HALT = 2'b11;

reg [1:0] state;

//------------------------------------------------
//         Packet Parsing Logic
//------------------------------------------------
// swap helper functions
function [15:0] swap16(input [15:0] in);
    swap16 = {in[7:0], in[15:8]};
endfunction

function [63:0] swap64(input [63:0] in);
    swap64 = {in[7:0], in[15:8], in[23:16], in[31:24],
               in[39:32], in[47:40], in[55:48], in[63:56]};
endfunction

// Fields
wire [15:0] w_ethertype_net = s_axis_tdata[111:96];
wire [15:0] w_ethertype =   swap16(w_ethertype_net);

wire [63:0] w_run_id_net = s_axis_tdata[175:112];
wire [63:0] w_rx_run_id = swap64(w_run_id_net);

wire [7:0] w_rx_knowledge_vec = s_axis_tdata[176+:8];

wire [7:0] w_rx_node_id = s_axis_tdata[184+:8];

wire [63:0] w_round_id_net = s_axis_tdata[192+:64];
wire [63:0] w_rx_round_id = swap64(w_round_id_net);

wire [(P_LOG_ITEM_LEN*8)-1:0] w_rx_payload_net = s_axis_tdata[256+:(P_LOG_ITEM_LEN*8)];
wire [(P_LOG_ITEM_LEN*8)-1:0] w_rx_payload = {
    swap64(w_rx_payload_net[63:0]),
    swap64(w_rx_payload_net[127:64]),
    swap64(w_rx_payload_net[191:128]),
    swap64(w_rx_payload_net[255:192])
};

// wire [7:0] w_rx_node_id = s_axis_if_rx_tid; // may be used instead
wire [7:0] w_rx_dest_id = s_axis_tdest;

//------------------------------------------------
//         Flitering Logic
//------------------------------------------------
reg r_packet_valid;

always @(*) begin // consensus core checks round and run ID
    r_packet_valid = 0;

    // Basic AXI Stream validity
    if (s_axis_tvalid && s_axis_tlast) begin
        // Check Ethertype
        if (w_ethertype == P_ETHERNET_TYPE) begin
            // Check Node ID within range
            if (w_rx_node_id < P_NODE_COUNT && w_rx_dest_id == P_NODE_ID && w_rx_run_id == i_current_run_id && w_rx_round_id == i_current_round_id) begin
                r_packet_valid = 1'b1;
            end
        end
    end
end

// round boundary logic

reg [63:0] prev_round_id;

wire new_round_pulse;
reg new_round_pulse_delayed;

assign new_round_pulse = (i_current_round_id != prev_round_id);

// payload matrix to store payloads from each node in the current round

reg [RAM_SEG_DATA_WIDTH-1:0] payload_matrix [0:P_NODE_COUNT-1] [0:MAX_COMMITS_PER_ROUND-1];
reg [RAM_SEG_DATA_WIDTH-1:0] payload_matrix_to_buffer [0:P_NODE_COUNT-1] [0:MAX_COMMITS_PER_ROUND-1];

reg [LOG_MAX_COMMITS-1:0] commit_count_per_node [0:P_NODE_COUNT-1];
reg [LOG_MAX_COMMITS-1:0] commit_count_per_node_to_buffer [0:P_NODE_COUNT-1];

reg [P_NODE_COUNT-1:0] committed_node_tracker;

//------------------------------------------------
//         Output Logic
//------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        o_rx_valid <= 0;
        o_rx_node_id <= 0;
        o_rx_sound_bitmap <= 0;
        o_rx_run_id <= 0;
        o_rx_round_id <= 0;

        commit_in_data <= 0;
        commit_in_be <= 0;
        commit_in_valid <= 0;
        commit_in_last <= 0;
        
        prev_round_id <= 0;
        new_round_pulse_delayed <= 0;
        committed_node_tracker <= 0;
    end else if (!i_rx_enabled) begin
        o_rx_valid <= 0;
        o_rx_node_id <= 0;
        o_rx_sound_bitmap <= 0;
        o_rx_run_id <= 0;
        o_rx_round_id <= 0;
        
        commit_in_data <= 0;
        commit_in_be <= 0;
        commit_in_valid <= 0;
        commit_in_last <= 0;
        prev_round_id <= 0;
        
        new_round_pulse_delayed <= 0;
        committed_node_tracker <= 0;
    end else begin
        prev_round_id <= i_current_round_id;
        new_round_pulse_delayed <= new_round_pulse;

        case (state)
            S_IDLE: begin
                if (i_rx_enabled) begin
                    state <= S_RECEIVE;
                end
                o_rx_node_id <= 0;
                o_rx_sound_bitmap <= 0;
                o_rx_run_id <= 0;
                o_rx_round_id <= 0;
            end

            S_RECEIVE: begin
                if (new_round_pulse_delayed) begin
                    state <= S_COMMIT;
                    payload_matrix_to_buffer <= payload_matrix;
                    commit_count_per_node_to_buffer <= commit_count_per_node;
                    // reset commit counts for the new round
                    payload_matrix <= '{default: '{default: 0}};
                    commit_count_per_node <= '{default: 0};
                    o_rx_node_id <= 0;
                    o_rx_sound_bitmap <= 0;
                    o_rx_run_id <= 0;
                    o_rx_round_id <= 0;
                    o_rx_valid <= 0;
                end else begin
                    if (r_packet_valid) begin
                        o_rx_node_id <= w_rx_node_id;
                        o_rx_sound_bitmap <= w_rx_knowledge_vec[P_NODE_COUNT-1:0];
                        o_rx_run_id <= w_rx_run_id[31:0];
                        o_rx_round_id <= w_rx_round_id[31:0];
                        o_rx_valid <= 1;

                        if (commit_count_per_node[w_rx_node_id] == 0) begin
                            payload_matrix[w_rx_node_id][0] <= w_rx_payload;
                            commit_count_per_node[w_rx_node_id] <= 1;
                        end else if ((commit_count_per_node[w_rx_node_id] < MAX_COMMITS_PER_ROUND) && (payload_matrix[w_rx_node_id][(commit_count_per_node[w_rx_node_id] - 1)] != w_rx_payload)) begin
                            payload_matrix[w_rx_node_id][commit_count_per_node[w_rx_node_id]] <= w_rx_payload;
                            commit_count_per_node[w_rx_node_id] <= commit_count_per_node[w_rx_node_id] + 1;
                        end // else begin
                        //     payload_matrix[w_rx_node_id][commit_count_per_node[w_rx_node_id]] <= payload_matrix[w_rx_node_id][commit_count_per_node[w_rx_node_id]];
                        //    commit_count_per_node[w_rx_node_id] <= commit_count_per_node[w_rx_node_id];
                        // end

                    end else begin
                        o_rx_node_id <= 0;
                        o_rx_sound_bitmap <= 0;
                        o_rx_run_id <= 0;
                        o_rx_round_id <= 0;
                        o_rx_valid <= 0;
                    end
                end
            end

            S_COMMIT: begin
                if (i_halt) begin
                    state <= S_HALT;
                end else begin
                    if (commit_count_per_node_to_buffer[committed_node_tracker] > 0 && committed_node_tracker != P_NODE_ID) begin
                        if (commit_in_ready) begin
                            commit_in_data <= payload_matrix_to_buffer[committed_node_tracker][0];
                            commit_in_be <= FULL_BE;
                            commit_in_last <= 1;
                            commit_in_valid <= 1;
                            // Shift the payloads for the current node to the left
                            for (int i = 0; i < MAX_COMMITS_PER_ROUND - 1; i++) begin
                                payload_matrix_to_buffer[committed_node_tracker][i] <= payload_matrix_to_buffer[committed_node_tracker][i + 1];
                            end
                            // Decrement the commit count for the current node
                            commit_count_per_node_to_buffer[committed_node_tracker] <= commit_count_per_node_to_buffer[committed_node_tracker] - 1;
                        end else begin
                            commit_in_data <= 0;
                            commit_in_be <= 0;
                            commit_in_valid <= 0;
                            commit_in_last <= 0;
                        end
                    end else begin
                        // Move to the next node
                        committed_node_tracker <= committed_node_tracker + 1;
                        if (committed_node_tracker == P_NODE_COUNT - 1) begin
                            state <= S_IDLE; // All nodes processed, go back to idle
                        end
                    end
                end

            end

            S_HALT: begin
                o_rx_node_id <= 0;
                o_rx_sound_bitmap <= 0;
                o_rx_run_id <= 0;
                o_rx_round_id <= 0;
                o_rx_valid <= 0;

                commit_in_data <= 0;
                commit_in_be <= 0;
                commit_in_valid <= 0;
                commit_in_last <= 0;
            end

            default: begin
                state <= S_IDLE;
            end

        endcase     
    end
end

endmodule
