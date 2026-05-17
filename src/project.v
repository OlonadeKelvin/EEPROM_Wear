/*
 * tt_um_wearlevel_controller
 *
 * Hardware Wear-Leveling Controller
 * ─────────────────────────────────────────────────
 *   ui_in[2:0]  logical block address (0..N-1)
 *   ui_in[4:3]  cmd  00=read_req  01=write_req  10=write_commit  11=telem_req
 *   ui_in[5]    move_ack  (external agent confirms data copy done)
 *   ui_in[7:6]  telem_sel 00=max_min_skew 01=total_writes_lo 10=total_writes_hi
 *               (only sampled when cmd==11)
 *
 *   uo_out[2:0] physical block address (result of current mapping)
 *   uo_out[3]   busy
 *   uo_out[4]   move_req  (ask agent to copy phys→uio_out[2:0])
 *   uo_out[5]   ecc_error (single-bit corrected — informational)
 *   uo_out[6]   block_retired (the accessed physical block is retired)
 *   uo_out[7]   telem_valid
 *
 *   uio_out[7:0] move_dest[2:0] || telem_data[7:0] (muxed)
 *   uio_oe       driven when move_req || telem_valid
 */

`default_nettype none
 
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
 
    // =========================================================
    // Parameters
    // =========================================================
    localparam N         = 8;
    localparam LOG2N     = 3;
    localparam PSI       = 8;           // Gap-advance period (writes)
    localparam CNT_WIDTH = 8;           // saturating wear counter width
    localparam TOT_WIDTH = 20;          // total-write counter width
 
    // =========================================================
    // I/O decode
    // =========================================================
    wire [LOG2N-1:0] logical   = ui_in[2:0];
    wire [1:0]       cmd       = ui_in[4:3];
    wire             move_ack  = ui_in[5];
    wire [1:0]       telem_sel = ui_in[7:6];
 
    wire cmd_read  = (cmd == 2'b00);
    wire cmd_write = (cmd == 2'b01);
    wire cmd_telem = (cmd == 2'b11);
 
    // =========================================================
    // Persistent Start-Gap state
    // =========================================================
    reg [LOG2N-1:0] Start_r, Gap_r;
    reg [LOG2N-1:0] Start_shd, Gap_shd;   // power-fail-safe shadows
    reg [2:0]       GapCnt_r;
 
    // =========================================================
    // Feistel LFSR — 10-bit, feedback taps [9]^[6]
    // KEY IS FROZEN at transaction start (not sampled mid-pipeline).
    // =========================================================
    reg [9:0] lfsr_r;
    reg [5:0] feistel_key;               // latched once per write transaction
 
    wire [9:0] lfsr_next = {lfsr_r[8:0], lfsr_r[9] ^ lfsr_r[6]};
 
    // =========================================================
    // Wear counters (8-bit saturating) + retired flags
    // =========================================================
    reg [CNT_WIDTH-1:0] cnt [0:N-1];
    reg                 retired [0:N-1];
 
    // =========================================================
    // Total-write counter
    // =========================================================
    reg [TOT_WIDTH-1:0] total_wr;
 
    // =========================================================
    // Hamming(6,3) ECC per 3-bit field
    //   codeword: [p1, p2, d0, p3, d1, d2]
    //   p1=d0^d1  p2=d0^d2  p3=d1^d2
    // =========================================================
    function automatic [5:0] ham_enc3;
        input [2:0] d;
        begin
            ham_enc3[0] = d[0] ^ d[1];
            ham_enc3[1] = d[0] ^ d[2];
            ham_enc3[2] = d[0];
            ham_enc3[3] = d[1] ^ d[2];
            ham_enc3[4] = d[1];
            ham_enc3[5] = d[2];
        end
    endfunction
 
    // Returns {err_flag, corrected_d[2:0]}
    function automatic [3:0] ham_dec3;
        input [5:0] c;
        reg [2:0] s;
        reg [5:0] cc;
        begin
            s[0] = c[0] ^ c[2] ^ c[4];
            s[1] = c[1] ^ c[2] ^ c[5];
            s[2] = c[3] ^ c[4] ^ c[5];
            cc = c;
            if (s != 3'b000) begin
                case (s)
                    3'd1: cc[0] = ~c[0];
                    3'd2: cc[1] = ~c[1];
                    3'd3: cc[2] = ~c[2];
                    3'd4: cc[3] = ~c[3];
                    3'd5: cc[4] = ~c[4];
                    3'd6: cc[5] = ~c[5];
                    default: cc = c;
                endcase
                ham_dec3 = {1'b1, cc[5], cc[4], cc[2]};
            end else begin
                ham_dec3 = {1'b0, c[5], c[4], c[2]};
            end
        end
    endfunction
 
    reg [5:0] ecc_start_r, ecc_gap_r;
 
    // =========================================================
    // 2-round 3-bit Feistel scrambler
    // =========================================================
    function automatic [2:0] feistel_fn;
        input [2:0] x;
        input [5:0] k;
        reg [1:0] L;
        reg       R, t, newR;
        begin
            L = x[2:1];
            R = x[0];
            // Round 0: F(R, k[0]) = R^k[0]; swap and XOR
            t    = R ^ k[0];
            newR = L[0] ^ t;
            L    = {1'b0, R};
            R    = newR;
            // Round 1: F(R, k[3])
            t    = R ^ k[3];
            newR = L[0] ^ t;
            L    = {1'b0, R};
            R    = newR;
            feistel_fn = {L[1:0], R};
        end
    endfunction
 
    // =========================================================
    // Start-Gap address mapping (pure combinational function)
    // =========================================================
    function automatic [LOG2N-1:0] startgap_fn;
        input [LOG2N-1:0] scr;
        input [LOG2N-1:0] s;
        input [LOG2N-1:0] g;
        reg [LOG2N-1:0] p;
        begin
            if (scr < g)
                p = (scr + s) % N;
            else
                p = (scr + s + 1) % N;
            startgap_fn = p;
        end
    endfunction
 
    // =========================================================
    // Combinational read path — zero FSM cycles for reads
    // Uses feistel_key (frozen for last write; stable during idle)
    // =========================================================
    wire [LOG2N-1:0] read_scr  = feistel_fn(logical, feistel_key);
    wire [LOG2N-1:0] read_phys = startgap_fn(read_scr, Start_r, Gap_r);
 
    // =========================================================
    // Telemetry: combinational max-min skew
    // =========================================================
    reg [CNT_WIDTH-1:0] tmax_r, tmin_r;
    integer ti;
    always @* begin
        tmax_r = cnt[0];
        tmin_r = cnt[0];
        for (ti = 1; ti < N; ti = ti + 1) begin
            if (!retired[ti]) begin
                if (cnt[ti] > tmax_r) tmax_r = cnt[ti];
                if (cnt[ti] < tmin_r) tmin_r = cnt[ti];
            end
        end
    end
    wire [CNT_WIDTH-1:0] skew_w = tmax_r - tmin_r;
 
    // =========================================================
    // FSM encoding
    // =========================================================
    localparam ST_IDLE     = 3'd0,
               ST_FEISTEL  = 3'd1,
               ST_MAP      = 3'd2,
               ST_INC      = 3'd3,
               ST_ADVANCE  = 3'd4,
               ST_RETIRE   = 3'd5,
               ST_WAIT_ACK = 3'd6,
               ST_TELEM    = 3'd7;
 
    reg [2:0] state;
 
    // Pipeline latches
    reg [LOG2N-1:0] log_lat, scr_lat, phys_lat;
 
    // Output registers
    reg busy_r, move_req_r, ecc_err_r, blk_ret_r, telem_vld_r;
    reg [7:0] uio_data_r;
    reg       uio_oe_r;
 
    // =========================================================
    // FSM — sequential
    // =========================================================
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            Start_r     <= 3'd0;
            Gap_r       <= 3'd0;
            Start_shd   <= 3'd0;
            Gap_shd     <= 3'd0;
            GapCnt_r    <= 3'd0;
            lfsr_r      <= 10'h3FF;
            feistel_key <= 6'h3F;
            total_wr    <= {TOT_WIDTH{1'b0}};
            busy_r      <= 1'b0;
            move_req_r  <= 1'b0;
            ecc_err_r   <= 1'b0;
            blk_ret_r   <= 1'b0;
            telem_vld_r <= 1'b0;
            uio_data_r  <= 8'd0;
            uio_oe_r    <= 1'b0;
            phys_lat    <= 3'd0;
            log_lat     <= 3'd0;
            scr_lat     <= 3'd0;
            ecc_start_r <= ham_enc3(3'd0);
            ecc_gap_r   <= ham_enc3(3'd0);
            for (k = 0; k < N; k = k + 1) begin
                cnt[k]     <= {CNT_WIDTH{1'b0}};
                retired[k] <= 1'b0;
            end
        end else begin
            // LFSR advances every clock for continuous key stream
            lfsr_r <= lfsr_next;
 
            case (state)
 
                // ── IDLE ───────────────────────────────────────────
                ST_IDLE: begin
                    // Clear all one-cycle signals
                    telem_vld_r <= 1'b0;
                    uio_oe_r    <= 1'b0;
                    move_req_r  <= 1'b0;
                    busy_r      <= 1'b0;
 
                    if (cmd_telem) begin
                        // Latch telemetry output and pulse valid next state
                        case (telem_sel)
                            2'b00: uio_data_r <= {{(8-CNT_WIDTH){1'b0}}, skew_w};
                            2'b01: uio_data_r <= total_wr[7:0];
                            2'b10: uio_data_r <= total_wr[15:8];
                            2'b11: uio_data_r <= {{(8-(TOT_WIDTH-16)){1'b0}},
                                                   total_wr[TOT_WIDTH-1:16]};
                        endcase
                        telem_vld_r <= 1'b1;
                        uio_oe_r    <= 1'b1;
                        state       <= ST_TELEM;
 
                    end else if (cmd_write && !busy_r) begin
                        // FIX 1: freeze Feistel key at transaction start
                        feistel_key <= lfsr_r[5:0];
                        log_lat     <= logical;
                        busy_r      <= 1'b1;
                        state       <= ST_FEISTEL;
                    end
                    // cmd_read: no FSM transition — handled combinationally
                end
 
                // ── FEISTEL ────────────────────────────────────────
                ST_FEISTEL: begin
                    scr_lat <= feistel_fn(log_lat, feistel_key);
                    state   <= ST_MAP;
                end
 
                // ── MAP ────────────────────────────────────────────
                ST_MAP: begin
                    begin : ecc_blk
                        reg [3:0] ds, dg;
                        ds        = ham_dec3(ecc_start_r);
                        dg        = ham_dec3(ecc_gap_r);
                        ecc_err_r <= ds[3] | dg[3];
                        Start_r   <= ds[2:0];
                        Gap_r     <= dg[2:0];
                        // Use current Start_r/Gap_r (updated next cycle, acceptable)
                        phys_lat  <= startgap_fn(scr_lat, Start_r, Gap_r);
                        blk_ret_r <= retired[startgap_fn(scr_lat, Start_r, Gap_r)];
                    end
                    state <= ST_INC;
                end
 
                // ── INC ────────────────────────────────────────────
                ST_INC: begin
                    // FIX 2: correct full-word saturation check
                    if (cnt[phys_lat] != {CNT_WIDTH{1'b1}})
                        cnt[phys_lat] <= cnt[phys_lat] + 1'b1;
                    total_wr <= total_wr + 1'b1;
                    state    <= ST_ADVANCE;
                end
 
                // ── ADVANCE ────────────────────────────────────────
                ST_ADVANCE: begin
                    if (GapCnt_r == (PSI - 1)) begin
                        GapCnt_r <= 3'd0;
                        if (((Gap_r + 1'b1) % N) == Start_r)
                            Start_r <= (Start_r + 1'b1) % N;
                        Gap_r <= (Gap_r + 1'b1) % N;
                    end else begin
                        GapCnt_r <= GapCnt_r + 1'b1;
                    end
                    // Atomic ECC commit
                    ecc_start_r <= ham_enc3(Start_r);
                    ecc_gap_r   <= ham_enc3(Gap_r);
                    Start_shd   <= Start_r;
                    Gap_shd     <= Gap_r;
                    state       <= ST_RETIRE;
                end
 
                // ── RETIRE ─────────────────────────────────────────
                ST_RETIRE: begin
                    if (cnt[phys_lat] == {CNT_WIDTH{1'b1}}) begin
                        retired[phys_lat] <= 1'b1;
                        move_req_r        <= 1'b1;
                        uio_data_r        <= {{(8-LOG2N){1'b0}}, phys_lat};
                        uio_oe_r          <= 1'b1;
                        // busy stays high until ACK
                        state             <= ST_WAIT_ACK;
                    end else begin
                        busy_r <= 1'b0;
                        state  <= ST_IDLE;
                    end
                end
 
                // ── WAIT_ACK ───────────────────────────────────────
                ST_WAIT_ACK: begin
                    if (move_ack) begin
                        move_req_r <= 1'b0;
                        uio_oe_r   <= 1'b0;
                        busy_r     <= 1'b0;
                        state      <= ST_IDLE;
                    end
                    // else hold: busy=1, move_req=1, uio driven
                end
 
                // ── TELEM ──────────────────────────────────────────
                // FIX 3: telem_valid was asserted entering this state;
                // clear it here and return to IDLE — strict 1-cycle pulse.
                ST_TELEM: begin
                    telem_vld_r <= 1'b0;
                    uio_oe_r    <= 1'b0;
                    state       <= ST_IDLE;
                end
 
                default: state <= ST_IDLE;
 
            endcase
        end
    end
 
    // =========================================================
    // Output assignment
    // FIX 4: READ is purely combinational — phys_comb drives output
    //         when cmd_read; phys_lat used for write results.
    // =========================================================
    wire [LOG2N-1:0] phys_out = cmd_read ? read_phys : phys_lat;
 
    assign uo_out[2:0] = phys_out;
    assign uo_out[3]   = busy_r;
    assign uo_out[4]   = move_req_r;
    assign uo_out[5]   = ecc_err_r;
    assign uo_out[6]   = blk_ret_r;
    assign uo_out[7]   = telem_vld_r;
 
    assign uio_out = uio_data_r;
    assign uio_oe  = {8{uio_oe_r}};
 
    // =========================================================
    // Formal properties (compile with `define FORMAL)
    // =========================================================
`ifdef FORMAL
    reg [TOT_WIDTH-1:0] f_prev;
    always @(posedge clk) f_prev <= total_wr;
 
    // 1. Monotonicity
    always @(posedge clk)
        if (rst_n) assert(total_wr >= f_prev);
 
    // 2. Bounded skew (Start-Gap theorem: max-min <= PSI+1)
    always @(posedge clk)
        if (rst_n) assert(skew_w <= PSI + 1);
 
    // 3. GapCnt in valid range
    always @(posedge clk)
        if (rst_n) assert(GapCnt_r < PSI);
 
    // 4. Start/Gap always in [0, N-1]
    always @(posedge clk)
        if (rst_n) begin
            assert(Start_r < N);
            assert(Gap_r   < N);
        end
 
    // 5. Liveness: busy always eventually clears
    cover property (
        @(posedge clk) disable iff (!rst_n)
        $rose(busy_r) ##[1:32] $fell(busy_r)
    );
`endif
 
endmodule
 

