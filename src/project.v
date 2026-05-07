/*
 * tt_um_wearlevel_controller - Hardware EEPROM Wear-Leveling Controller
 *
 * A fully digital block that implements dynamic wear-leveling for an external
 * EEPROM or flash. It tracks write counts per physical block and automatically
 * remaps logical‑to‑physical addresses to prevent localised oxide wear.
 *
 * Interface:
 *   ui_in[2:0]  – logical block address (up to 8 blocks)
 *   ui_in[4:3]  – command (00=read_req, 01=write_req, 10=write_commit, 11=move_ack)
 *   ui_in[5]    – move_ack (used when move_request is active)
 *   uo_out[2:0] – physical block address
 *   uo_out[3]   – busy flag (active high during a write‑commit sequence)
 *   uo_out[4]   – move_request (active high when a block swap needs data movement)
 *   uio_out[2:0]– destination physical address during a move (otherwise 0)
 *   uio_oe[2:0] – output enable for uio_out (0b111 during move, else 0)
 *
 * The controller performs dynamic wear‑levelling:
 *   - On a write_commit it increments the write counter of the current physical block.
 *   - If the difference between that counter and the minimum counter among all
 *     physical blocks exceeds a fixed threshold (4), it remaps the two blocks
 *     to balance wear and asserts move_request.
 */

module tt_um_wearlevel_controller (
    input  wire [7:0] ui_in,    // {move_ack, 1'b0, cmd[1:0], addr[2:0]} (ui_in[5]=move_ack)
    output wire [7:0] uo_out,   // {2'b00, move_request, busy, phys_addr[2:0]}
    input  wire [7:0] uio_in,   // unused
    output wire [7:0] uio_out,  // destination physical address during move
    output wire [7:0] uio_oe,   // 3'b111 during move, else 3'b000
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------
    parameter NUM_BLOCKS  = 8;
    parameter THRESHOLD   = 4;
    parameter CNT_WIDTH   = 16;

    // State encoding
    localparam IDLE       = 3'd0,
               S_INC      = 3'd1,
               S_MIN      = 3'd2,
               S_CHECK    = 3'd3,
               S_FIND_LOG = 3'd4,
               S_SWAP     = 3'd5,
               S_WAIT_ACK = 3'd6;

    // ------------------------------------------------------------
    // Internal signals
    // ------------------------------------------------------------
    wire [2:0] addr       = ui_in[2:0];
    wire [1:0] cmd        = ui_in[4:3];
    wire       move_ack   = ui_in[5];

    // Command decoding
    wire cmd_read_req   = (cmd == 2'b00);
    wire cmd_write_req  = (cmd == 2'b01);
    wire cmd_write_commit = (cmd == 2'b10);

    // ------------------------------------------------------------
    // Register arrays : mapping and wear counters
    // ------------------------------------------------------------
    reg [2:0] map [0:NUM_BLOCKS-1];         // logical -> physical
    reg [CNT_WIDTH-1:0] wr_count [0:NUM_BLOCKS-1]; // wear count per physical block

    // FSM state and control registers
    reg [2:0] state, next_state;
    reg       write_pending;
    reg [2:0] last_logical, last_physical;   // from most recent write_req

    // Registered outputs
    reg [2:0] phys_out;
    reg       busy, move_req;
    reg [2:0] uio_data;
    reg [2:0] uio_oe_reg;

    // Min‑search registers
    reg [CNT_WIDTH-1:0] min_val_reg;
    reg [2:0] min_idx_reg;
    reg [2:0] swap_logical_reg;
    reg       remap_needed;

    // ------------------------------------------------------------
    // Combinational: current physical for a request
    // ------------------------------------------------------------
    wire [2:0] req_phys = map[addr];   // used only when idle and read/write_req

    // ------------------------------------------------------------
    // FSM sequential block
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            write_pending <= 1'b0;
            busy <= 1'b0;
            move_req <= 1'b0;
            uio_oe_reg <= 3'b000;
            uio_data <= 3'b000;
            min_val_reg <= 0;
            min_idx_reg <= 0;
            swap_logical_reg <= 0;
            remap_needed <= 1'b0;
            phys_out <= 3'd0;
            // initialise mapping and counters
            integer i;
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

            // Write commit triggers FSM
            if (state == IDLE && cmd_write_commit && write_pending) begin
                write_pending <= 1'b0;
            end

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    move_req <= 1'b0;
                    uio_oe_reg <= 3'b000;
                    // update physical output for read/write_req
                    if (cmd_read_req || cmd_write_req)
                        phys_out <= req_phys;
                end

                S_INC: begin
                    // increment wear counter of last physical block
                    wr_count[last_physical] <= wr_count[last_physical] + 1;
                    busy <= 1'b1;
                end

                S_MIN: begin
                    // latch the minimum wear counter and its index
                    min_val_reg <= min_val_comb;
                    min_idx_reg <= min_idx_comb;
                    busy <= 1'b1;
                end

                S_CHECK: begin
                    if ( (wr_count[last_physical] - min_val_reg) > THRESHOLD )
                        remap_needed <= 1'b1;
                    else
                        remap_needed <= 1'b0;
                    busy <= 1'b1;
                end

                S_FIND_LOG: begin
                    swap_logical_reg <= swap_logical_comb;
                    busy <= 1'b1;
                end

                S_SWAP: begin
                    // swap mappings
                    map[last_logical] <= min_idx_reg;
                    map[swap_logical_reg] <= last_physical;
                    move_req <= 1'b1;
                    uio_oe_reg <= 3'b111;
                    uio_data <= min_idx_reg;     // destination physical block
                    phys_out <= last_physical;   // source physical block
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

                default: begin
                    busy <= 1'b0;
                end
            endcase
        end
    end

    // ------------------------------------------------------------
    // Combinational min search (8 elements, 16‑bit)
    // ------------------------------------------------------------
    reg [CNT_WIDTH-1:0] m_val;
    reg [2:0]           m_idx;
    integer k;

    always @* begin
        m_val = wr_count[0];
        m_idx = 3'd0;
        for (k = 1; k < 8; k = k+1) begin
            if (wr_count[k] < m_val) begin
                m_val = wr_count[k];
                m_idx = k[2:0];
            end
        end
    end

    wire [CNT_WIDTH-1:0] min_val_comb = m_val;
    wire [2:0]           min_idx_comb = m_idx;

    // ------------------------------------------------------------
    // Combinational find logical block mapped to min_idx_reg
    // ------------------------------------------------------------
    reg [2:0] sw_log;
    integer j;

    always @* begin
        sw_log = 3'd0;
        for (j = 0; j < 8; j = j+1) begin
            if (map[j] == min_idx_reg)
                sw_log = j[2:0];
        end
    end
    wire [2:0] swap_logical_comb = sw_log;

    // ------------------------------------------------------------
    // Next state logic
    // ------------------------------------------------------------
    always @* begin
        next_state = state;
        case (state)
            IDLE: begin
                if (write_pending && cmd_write_commit)
                    next_state = S_INC;
                else
                    next_state = IDLE;
            end
            S_INC:      next_state = S_MIN;
            S_MIN:      next_state = S_CHECK;
            S_CHECK:    next_state = remap_needed ? S_FIND_LOG : IDLE;
            S_FIND_LOG: next_state = S_SWAP;
            S_SWAP:     next_state = S_WAIT_ACK;
            S_WAIT_ACK: next_state = (move_ack) ? IDLE : S_WAIT_ACK;
            default:    next_state = IDLE;
        endcase
    end

    // ------------------------------------------------------------
    // Output assignments
    // ------------------------------------------------------------
    assign uo_out[2:0] = phys_out;
    assign uo_out[3]   = busy;
    assign uo_out[4]   = move_req;
    assign uo_out[7:5] = 3'b000;   // unused

    assign uio_out[2:0] = uio_data;
    assign uio_out[7:3] = 5'b00000;
    assign uio_oe[2:0]  = uio_oe_reg;
    assign uio_oe[7:3]  = 5'b00000;

endmodule
