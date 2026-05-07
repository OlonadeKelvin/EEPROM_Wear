/*
 * tt_um_wearlevel_controller - Hardware EEPROM Wear-Leveling Controller
 *
 * Single-tile digital IP for dynamic wear-leveling of external EEPROM/flash.
 * Tracks write counts per physical block and automatically remaps logical
 * addresses when a wear imbalance exceeds a fixed threshold.
 */

module tt_um_wearlevel_controller (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // Parameters
    localparam NUM_BLOCKS = 4;
    localparam THRESHOLD  = 4;
    localparam CNT_WIDTH  = 16;

    // States
    localparam IDLE       = 3'd0,
               S_INC      = 3'd1,
               S_MIN      = 3'd2,
               S_CHECK    = 3'd3,
               S_FIND_LOG = 3'd4,
               S_SWAP     = 3'd5,
               S_WAIT_ACK = 3'd6;

    // Registers for internal state
    reg [2:0] map [0:NUM_BLOCKS-1];                 // logical -> physical
    reg [CNT_WIDTH-1:0] wr_count [0:NUM_BLOCKS-1];  // wear counters

    reg [2:0] state, next_state;
    reg       write_pending;
    reg [2:0] last_logical, last_physical;
    reg [2:0] phys_out;
    reg       busy, move_req;
    reg [2:0] uio_data;
    reg [2:0] uio_oe_reg;

    // Min-search storage
    reg [CNT_WIDTH-1:0] min_val_reg;
    reg [2:0]           min_idx_reg;
    reg [2:0]           swap_logical_reg;
    reg                 remap_needed;

    // Command decoding
    wire [2:0] addr = ui_in[2:0];
    wire [1:0] cmd  = ui_in[4:3];
    wire       move_ack = ui_in[5];

    wire cmd_read_req   = (cmd == 2'b00);
    wire cmd_write_req  = (cmd == 2'b01);
    wire cmd_write_commit = (cmd == 2'b10);

    // Current physical address for a read/write request
    wire [2:0] req_phys = map[addr];

    // --------------------------------------------------------
    // Combinational min-search (unrolled for 4 blocks)
    // --------------------------------------------------------
    wire [CNT_WIDTH-1:0] cnt0 = wr_count[0];
    wire [CNT_WIDTH-1:0] cnt1 = wr_count[1];
    wire [CNT_WIDTH-1:0] cnt2 = wr_count[2];
    wire [CNT_WIDTH-1:0] cnt3 = wr_count[3];

    // Cascade comparison
    wire [CNT_WIDTH-1:0] min01 = (cnt0 < cnt1) ? cnt0 : cnt1;
    wire [2:0]           idx01 = (cnt0 < cnt1) ? 3'd0 : 3'd1;

    wire [CNT_WIDTH-1:0] min23 = (cnt2 < cnt3) ? cnt2 : cnt3;
    wire [2:0]           idx23 = (cnt2 < cnt3) ? 3'd2 : 3'd3;

    wire [CNT_WIDTH-1:0] min_val_comb = (min01 < min23) ? min01 : min23;
    wire [2:0]           min_idx_comb = (min01 < min23) ? idx01 : idx23;

    // --------------------------------------------------------
    // Combinational find logical block for a physical index
    // --------------------------------------------------------
    reg [2:0] swap_logical;
    integer j;
    integer i;
    always @* begin
        swap_logical = 3'd0;
        for (j = 0; j < NUM_BLOCKS; j = j+1)
            if (map[j] == min_idx_reg)
                swap_logical = j[2:0];
    end

    // --------------------------------------------------------
    // Sequential FSM
    // --------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            write_pending <= 1'b0;
            busy <= 1'b0;
            move_req <= 1'b0;
            uio_oe_reg <= 3'b000;
            uio_data <= 3'b000;
            phys_out <= 3'd0;
            min_val_reg <= 0;
            min_idx_reg <= 0;
            swap_logical_reg <= 0;
            remap_needed <= 1'b0;
            // initialise mapping and counters
            for (i = 0; i < NUM_BLOCKS; i = i+1) begin
                map[i] <= i[2:0];
                wr_count[i] <= 0;
            end
        end else begin
            state <= next_state;

            // Capture write request
            if (state == IDLE && cmd_write_req && !busy) begin
                last_logical <= addr;
                last_physical <= req_phys;
                write_pending <= 1'b1;
            end

            if (state == IDLE && cmd_write_commit && write_pending) begin
                write_pending <= 1'b0;
            end

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    move_req <= 1'b0;
                    uio_oe_reg <= 3'b000;
                    if (cmd_read_req || cmd_write_req)
                        phys_out <= req_phys;
                end

                S_INC: begin
                    wr_count[last_physical] <= wr_count[last_physical] + 1;
                    busy <= 1'b1;
                end

                S_MIN: begin
                    min_val_reg <= min_val_comb;
                    min_idx_reg <= min_idx_comb;
                    busy <= 1'b1;
                end

                S_CHECK: begin
                    if ((wr_count[last_physical] - min_val_reg) > THRESHOLD)
                        remap_needed <= 1'b1;
                    else
                        remap_needed <= 1'b0;
                    busy <= 1'b1;
                end

                S_FIND_LOG: begin
                    swap_logical_reg <= swap_logical;
                    busy <= 1'b1;
                end

                S_SWAP: begin
                    map[last_logical] <= min_idx_reg;
                    map[swap_logical_reg] <= last_physical;
                    move_req <= 1'b1;
                    uio_oe_reg <= 3'b111;
                    uio_data <= min_idx_reg;
                    phys_out <= last_physical;
                    busy <= 1'b1;
                end

                S_WAIT_ACK: begin
                    if (move_ack) begin
                        move_req <= 1'b0;
                        uio_oe_reg <= 3'b000;
                        busy <= 1'b0;
                    end else begin
                        busy <= 1'b1;
                    end
                end

                default: busy <= 1'b0;
            endcase
        end
    end

    // Next state logic
    always @* begin
        next_state = state;
        case (state)
            IDLE:       next_state = (write_pending && cmd_write_commit) ? S_INC : IDLE;
            S_INC:      next_state = S_MIN;
            S_MIN:      next_state = S_CHECK;
            S_CHECK:    next_state = remap_needed ? S_FIND_LOG : IDLE;
            S_FIND_LOG: next_state = S_SWAP;
            S_SWAP:     next_state = S_WAIT_ACK;
            S_WAIT_ACK: next_state = move_ack ? IDLE : S_WAIT_ACK;
            default:    next_state = IDLE;
        endcase
    end

    assign uo_out[2:0] = phys_out;
    assign uo_out[3]   = busy;
    assign uo_out[4]   = move_req;
    assign uo_out[7:5] = 3'b000;

    assign uio_out[2:0] = uio_data;
    assign uio_out[7:3] = 5'b00000;
    assign uio_oe[2:0]  = uio_oe_reg;
    assign uio_oe[7:3]  = 5'b00000;

endmodule
