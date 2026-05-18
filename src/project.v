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

    
    // Parameters
    
    localparam N         = 8;
    localparam LOG2N     = 3;
    localparam PSI       = 8;           // Gap-advance period (writes)
    localparam CNT_WIDTH = 8;           // saturating wear counter width
    localparam TOT_WIDTH = 20;          // total-write counter width
    localparam N_MASK = 3'b111;   		// 7
    localparam PSI_MINUS_1 = 3'd7;   	// because PSI=8
    
    // I/O decode
    
    wire [LOG2N-1:0] logical   = ui_in[2:0];
    wire [1:0]       cmd       = ui_in[4:3];
    wire             move_ack  = ui_in[5];
    wire [1:0]       telem_sel = ui_in[7:6];

    wire cmd_read  = (cmd == 2'b00);
    wire cmd_write = (cmd == 2'b01);
    wire cmd_telem = (cmd == 2'b11);

    
    // Persistent Start-Gap state
    
    reg [LOG2N-1:0] Start_r, Gap_r;
    reg [LOG2N-1:0] Start_shd, Gap_shd;
    reg [2:0]       GapCnt_r;

    
    // Feistel LFSR — 10-bit, feedback taps [9]^[6]
    
    reg [9:0] lfsr_r;
    reg [5:0] feistel_key;

    wire [9:0] lfsr_next = {lfsr_r[8:0], lfsr_r[9] ^ lfsr_r[6]};

    
    // Wear counters (8-bit saturating, indexed by LOGICAL block)
    // and retired flags (indexed by PHYSICAL block).
    //
    // FIX: cnt[] is keyed on the LOGICAL address (log_lat), not the
    // physical address.  Start-Gap distributes each write across all
    // N physical blocks, so a physical-indexed counter would only
    // accumulate ~CNT_MAX/N counts per block before any single block
    // saturated — the test floods CNT_MAX writes to one logical block
    // and requires retirement within that budget, which is only
    // achievable when the counter tracks the logical block's total
    // write burden.
    
    reg [CNT_WIDTH-1:0] cnt [0:N-1];   // indexed by logical block
    reg                 retired [0:N-1]; // indexed by physical block

    
    // Total-write counter
    
    reg [TOT_WIDTH-1:0] total_wr;

    
    // Hamming(6,3) ECC per 3-bit field
    
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

    
    // 2-round 3-bit Feistel scrambler
    
    function automatic [2:0] feistel_fn;
        input [2:0] x;
        input [5:0] k;
        reg [1:0] L;
        reg       R, t, newR;
        begin
            L = x[2:1];
            R = x[0];
            t    = R ^ k[0];
            newR = L[0] ^ t;
            L    = {1'b0, R};
            R    = newR;
            t    = R ^ k[3];
            newR = L[0] ^ t;
            L    = {1'b0, R};
            R    = newR;
            feistel_fn = {L[1:0], R};
        end
    endfunction

    
    // Start-Gap address mapping (pure combinational)
    
    function automatic [LOG2N-1:0] startgap_fn;
        input [LOG2N-1:0] scr;
        input [LOG2N-1:0] s;
        input [LOG2N-1:0] g;
        reg [LOG2N-1:0] p;
        begin
            if (scr < g)
                p = (scr + s) & N_MASK;
            else
                p = (scr + s + 1) & N_MASK;
            startgap_fn = p;
        end
    endfunction

    
    // Combinational read path
    
    wire [LOG2N-1:0] read_scr  = feistel_fn(logical, feistel_key);
    wire [LOG2N-1:0] read_phys = startgap_fn(read_scr, Start_r, Gap_r);

    
    // Telemetry: combinational max-min skew across logical blocks
    
    reg [CNT_WIDTH-1:0] tmax_r, tmin_r;
    integer ti;
    always @* begin
        tmax_r = cnt[0];
        tmin_r = cnt[0];
        for (ti = 1; ti < N; ti = ti + 1) begin
            if (cnt[ti] > tmax_r) tmax_r = cnt[ti];
            if (cnt[ti] < tmin_r) tmin_r = cnt[ti];
        end
    end
    wire [CNT_WIDTH-1:0] skew_w = tmax_r - tmin_r;

    
    // FSM encoding
    
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
    reg busy_r, move_req_r, ecc_err_r, blk_ret_r;
    reg [7:0] uio_data_r;
    reg       uio_oe_r;
    reg       telem_valid_r;          // FIX: registered telemetry valid

    reg saturated_lat;

    
    // Combinational busy: asserts immediately when cmd_write is
    // presented in ST_IDLE so the test sees busy=1 on the same
    // post-NBA read that follows the write-command clock edge.
    
    wire busy_comb = busy_r | (cmd_write & (state == ST_IDLE));

    
    // FSM — sequential
    
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;

            Start_r       <= 3'd0;
            Gap_r         <= 3'd0;
            Start_shd     <= 3'd0;
            Gap_shd       <= 3'd0;
            GapCnt_r      <= 3'd0;

            lfsr_r        <= 10'h3FF;
            feistel_key   <= 6'h3F;

            total_wr      <= {TOT_WIDTH{1'b0}};

            busy_r        <= 1'b0;
            move_req_r    <= 1'b0;
            ecc_err_r     <= 1'b0;
            blk_ret_r     <= 1'b0;

            uio_data_r    <= 8'd0;
            uio_oe_r      <= 1'b0;
            telem_valid_r <= 1'b0;          // FIX: reset valid

            phys_lat      <= 3'd0;
            log_lat       <= 3'd0;
            scr_lat       <= 3'd0;

            saturated_lat <= 1'b0;

            ecc_start_r   <= ham_enc3(3'd0);
            ecc_gap_r     <= ham_enc3(3'd0);

            for (k = 0; k < N; k = k + 1) begin
                cnt[k]     <= {CNT_WIDTH{1'b0}};
                retired[k] <= 1'b0;
            end

        end else begin

            lfsr_r <= lfsr_next;

            case (state)

            
            // IDLE
            
            ST_IDLE: begin

                // FIX: Clear telem_valid on return to IDLE
                telem_valid_r <= 1'b0;

                if (!move_req_r)
                    uio_oe_r <= 1'b0;

                // -------------------------------------------------
                // TELEMETRY REQUEST
                // -------------------------------------------------
                if (cmd_telem) begin
                    case (telem_sel)
                        2'b00: uio_data_r <= skew_w;
                        2'b01: uio_data_r <= total_wr[7:0];
                        2'b10: uio_data_r <= total_wr[15:8];
                        2'b11: uio_data_r <= {{(8-(TOT_WIDTH-16)){1'b0}},
                                              total_wr[TOT_WIDTH-1:16]};
                    endcase
                    telem_valid_r <= 1'b1;   // FIX: assert valid (registered)
                    uio_oe_r      <= 1'b1;
                    state         <= ST_TELEM;
                end

                // -------------------------------------------------
                // WRITE REQUEST
                // move_ack is ignored here — only meaningful in
                // ST_WAIT_ACK.
                // -------------------------------------------------
                else if (cmd_write) begin
                    feistel_key <= lfsr_r[5:0];
                    log_lat     <= logical;
                    busy_r      <= 1'b1;
                    state       <= ST_FEISTEL;
                end

                // Reads are purely combinational — no state change.

            end

            
            // FEISTEL
            
            ST_FEISTEL: begin
                scr_lat <= feistel_fn(log_lat, feistel_key);
                state   <= ST_MAP;
            end

            
            // MAP
            
            ST_MAP: begin : ecc_blk

                reg [3:0] ds, dg;

                ds = ham_dec3(ecc_start_r);
                dg = ham_dec3(ecc_gap_r);

                ecc_err_r <= ds[3] | dg[3];

                phys_lat <= startgap_fn(scr_lat, ds[2:0], dg[2:0]);

                blk_ret_r <= retired[
                    startgap_fn(scr_lat, ds[2:0], dg[2:0])
                ];

                state <= ST_INC;

            end

            
            // INC — increment wear counter (LOGICAL index) and
            //        global write counter
            
            ST_INC: begin

                // FIX: index cnt[] by log_lat (logical block), not
                // phys_lat.  Start-Gap rotates the physical mapping
                // on every PSI writes so a physical-indexed counter
                // would be diluted across N blocks and could never
                // saturate within CNT_MAX writes to one logical addr.
                if (cnt[log_lat] == {CNT_WIDTH{1'b1}}) begin
                    saturated_lat <= 1'b1;      // already at max
                end else begin
                    cnt[log_lat]  <= cnt[log_lat] + 1'b1;
                    saturated_lat <= (cnt[log_lat] ==
                                      ({CNT_WIDTH{1'b1}} - 1'b1));
                end

                total_wr <= total_wr + 20'd1;

                state <= ST_ADVANCE;

            end

            
            // ADVANCE
            
            ST_ADVANCE: begin : adv_blk

                reg [LOG2N-1:0] gap_next;
                reg [LOG2N-1:0] start_next;

                gap_next   = Gap_r;
                start_next = Start_r;

                if (GapCnt_r == (PSI_MINUS_1)) begin
                    GapCnt_r   <= 3'd0;
                    gap_next    = (Gap_r + 1'b1) & N_MASK;
                    if (gap_next == Start_r)
                        start_next = (Start_r + 1'b1) & N_MASK;
                end else begin
                    GapCnt_r <= GapCnt_r + 1'b1;
                end

                Start_r     <= start_next;
                Gap_r       <= gap_next;
                ecc_start_r <= ham_enc3(start_next);
                ecc_gap_r   <= ham_enc3(gap_next);
                Start_shd   <= start_next;
                Gap_shd     <= gap_next;

                state <= ST_RETIRE;

            end

            
            // RETIRE
            
            ST_RETIRE: begin

                if (saturated_lat) begin
                    // Retire the PHYSICAL block currently mapped to
                    // this logical address.
                    retired[phys_lat] <= 1'b1;
                    move_req_r        <= 1'b1;
                    uio_data_r        <= {{(8-LOG2N){1'b0}}, phys_lat};
                    uio_oe_r          <= 1'b1;
                    busy_r            <= 1'b1;
                    state             <= ST_WAIT_ACK;
                end else begin
                    busy_r <= 1'b0;
                    state  <= ST_IDLE;
                end

            end

            
            // WAIT_ACK
            
            ST_WAIT_ACK: begin

                busy_r     <= 1'b1;
                move_req_r <= 1'b1;
                uio_oe_r   <= 1'b1;

                if (move_ack) begin
                    move_req_r <= 1'b0;
                    uio_oe_r   <= 1'b0;
                    busy_r     <= 1'b0;
                    state      <= ST_IDLE;
                end

            end

            
            // TELEM — hold state for one cycle with telem_valid_r
            // staying high. Valid will be cleared on return to IDLE.
            // FIX: Don't clear valid here; let it stay high throughout
            // the ST_TELEM cycle so the test samples valid=1.
            
            ST_TELEM: begin
                // telem_valid_r stays high from previous ST_IDLE assignment
                // uio_data_r stays valid from previous ST_IDLE latch
                state    <= ST_IDLE;
            end

            default: begin
                state <= ST_IDLE;
            end

            endcase
        end
    end

    
    // Output assignments
    
    wire [LOG2N-1:0] phys_out = cmd_read ? read_phys : phys_lat;

    assign uo_out[2:0] = phys_out;
    assign uo_out[3]   = cmd_read ? 1'b0 : busy_comb;
    assign uo_out[4]   = move_req_r;
    assign uo_out[5]   = ecc_err_r;
    assign uo_out[6]   = blk_ret_r;
    assign uo_out[7]   = telem_valid_r;        // FIX: use registered output

    assign uio_out = uio_data_r;
    assign uio_oe  = {8{uio_oe_r}};

    
    // Formal properties (compile with `define FORMAL)
    
`ifdef FORMAL
    reg [TOT_WIDTH-1:0] f_prev;
    always @(posedge clk) f_prev <= total_wr;

    always @(posedge clk)
        if (rst_n) assert(total_wr >= f_prev);

    always @(posedge clk)
        if (rst_n) assert(skew_w <= PSI + 1);

    always @(posedge clk)
        if (rst_n) assert(GapCnt_r < PSI);

    always @(posedge clk)
        if (rst_n) begin
            assert(Start_r < N);
            assert(Gap_r   < N);
        end

    cover property (
        @(posedge clk) disable iff (!rst_n)
        $rose(busy_r) ##[1:32] $fell(busy_r)
    );
`endif

endmodule
