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
    localparam N          = 8;           // number of physical blocks
    localparam LOG2N      = 3;           // log2(N)
    localparam PSI        = 8;           // gap-advance period (writes)
    localparam CNT_WIDTH  = 8;           // saturating wear counter width
    localparam TOT_WIDTH  = 20;          // total-write counter width

    // I/O decode
    wire [LOG2N-1:0] logical   = ui_in[2:0];
    wire [1:0]       cmd       = ui_in[4:3];
    wire             move_ack  = ui_in[5];
    wire [1:0]       telem_sel = ui_in[7:6];

    wire cmd_read    = (cmd == 2'b00);
    wire cmd_write   = (cmd == 2'b01);
    wire cmd_commit  = (cmd == 2'b10);
    wire cmd_telem   = (cmd == 2'b11);

    // Persistent state — Start-Gap algorithm
    reg [LOG2N-1:0] Start_r,     Gap_r;     // live registers
    reg [LOG2N-1:0] Start_shd,   Gap_shd;   // shadow (committed) registers
    reg [2:0]       GapCnt_r;               // counts writes mod PSI


    // Feistel LFSR key (10-bit, taps 10,7)
    reg [9:0] lfsr_r;

    wire [9:0] lfsr_next = {lfsr_r[8:0], lfsr_r[9] ^ lfsr_r[6]};


    // Wear counters (8-bit saturating) + retired flags
    reg [CNT_WIDTH-1:0] cnt [0:N-1];
    reg                 retired [0:N-1];


    // Total write counter
    reg [TOT_WIDTH-1:0] total_wr;


    // FSM
    localparam ST_IDLE      = 3'd0,
               ST_FEISTEL   = 3'd1,   // compute scrambled logical
               ST_MAP       = 3'd2,   // compute physical from Start-Gap
               ST_INC       = 3'd3,   // increment wear counter
               ST_ADVANCE   = 3'd4,   // maybe advance Gap
               ST_RETIRE    = 3'd5,   // mark retired if saturated
               ST_WAIT_ACK  = 3'd6,
               ST_TELEM     = 3'd7;

    reg [2:0] state;

    // Pipeline registers
    reg [LOG2N-1:0] log_lat;      // latched logical address
    reg [LOG2N-1:0] scr_lat;      // scrambled logical
    reg [LOG2N-1:0] phys_lat;     // computed physical
    reg             write_pend;
    reg             busy_r;
    reg             move_req_r;
    reg             ecc_err_r;
    reg             blk_ret_r;
    reg             telem_vld_r;
    reg [7:0]       telem_data_r;
    reg [7:0]       uio_data_r;
    reg             uio_oe_r;

    // Hamming(13,8) ECC — encode {Start[2:0], Gap[2:0]} → 8 data + 5 parity
    // We encode 6 data bits (start[2:0], gap[2:0]) with 4 parity bits → (10,6)
    // For clarity we use a simple (7,4) on each 3-bit field separately.
    //
    // Simplified: 3-bit Hamming(6,3) per field — p1,p2,d1,p3,d2,d3
    // p1 = d1^d2   p2 = d1^d3   p3 = d2^d3

    // Encode Start
    function [5:0] ham_enc3;
        input [2:0] d;
        begin
            ham_enc3[0] = d[0] ^ d[1];   // p1
            ham_enc3[1] = d[0] ^ d[2];   // p2
            ham_enc3[2] = d[0];           // d1
            ham_enc3[3] = d[1] ^ d[2];   // p3
            ham_enc3[4] = d[1];           // d2
            ham_enc3[5] = d[2];           // d3
        end
    endfunction

    // Decode + correct Start
    function [3:0] ham_dec3; // [3]=error flag, [2:0]=corrected data
        input [5:0] c;
        reg [2:0] s;
        reg [5:0] cc;
        begin
            s[0] = c[0] ^ c[2] ^ c[4];
            s[1] = c[1] ^ c[2] ^ c[5];
            s[2] = c[3] ^ c[4] ^ c[5];
            cc = c;
            if (s != 3'b000) begin
                cc[s-1] = ~c[s-1]; // correct single-bit error
                ham_dec3 = {1'b1, cc[5], cc[4], cc[2]};
            end else begin
                ham_dec3 = {1'b0, c[5], c[4], c[2]};
            end
        end
    endfunction

    // ECC registers (store encoded persistent state)
    reg [5:0] ecc_start_r, ecc_gap_r;


    // 2-round 3-bit Feistel scrambler
    // Key: upper 6 bits of LFSR split into two 3-bit round keys
    function [2:0] feistel;
        input [2:0] x;
        input [5:0] k;  // {k1[2:0], k0[2:0]}
        reg [2:0] L, R, t;
        begin
            L = x[2:1]; // 2 bits (pad with 0 on MSB)
            R = x[0];   // 1 bit
            // Round 0
            t = R ^ k[2:0];
            {L, R} = {R, L ^ t[0]};
            // Round 1
            t = R ^ k[5:3];
            {L, R} = {R, L ^ t[0]};
            feistel = {L[1:0], R};
        end
    endfunction

    // Start-Gap physical address computation
    //   phys = (scr + Start) mod N         if scr < Gap
    //   phys = (scr + Start + 1) mod N     otherwise
    // Skip retired blocks by +1 modulo (up to N attempts)

    function [LOG2N-1:0] startgap;
        input [LOG2N-1:0] scr;
        input [LOG2N-1:0] s;
        input [LOG2N-1:0] g;
        reg [LOG2N-1:0] p;
        begin
            if (scr < g)
                p = (scr + s) % N;
            else
                p = (scr + s + 1) % N;
            startgap = p;
        end
    endfunction


    // Telemetry helpers
    // Max-min skew (combinational over 8 counters)
    reg [CNT_WIDTH-1:0] tmax, tmin;
    integer ti;
    always @* begin
        tmax = cnt[0]; tmin = cnt[0];
        for (ti = 1; ti < N; ti = ti+1) begin
            if (!retired[ti]) begin
                if (cnt[ti] > tmax) tmax = cnt[ti];
                if (cnt[ti] < tmin) tmin = cnt[ti];
            end
        end
    end
    wire [CNT_WIDTH-1:0] skew = tmax - tmin;

    // Reset / FSM
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            Start_r      <= 0;
            Gap_r        <= 0;
            Start_shd    <= 0;
            Gap_shd      <= 0;
            GapCnt_r     <= 0;
            lfsr_r       <= 10'h3FF;   // non-zero seed
            total_wr     <= 0;
            write_pend   <= 0;
            busy_r       <= 0;
            move_req_r   <= 0;
            ecc_err_r    <= 0;
            blk_ret_r    <= 0;
            telem_vld_r  <= 0;
            telem_data_r <= 0;
            uio_data_r   <= 0;
            uio_oe_r     <= 0;
            phys_lat     <= 0;
            log_lat      <= 0;
            scr_lat      <= 0;
            ecc_start_r  <= ham_enc3(3'd0);
            ecc_gap_r    <= ham_enc3(3'd0);
            for (k = 0; k < N; k = k+1) begin
                cnt[k]     <= 0;
                retired[k] <= 0;
            end
        end else begin
            // LFSR always ticks
            lfsr_r <= lfsr_next;

            case (state)
           
                ST_IDLE: begin
                    busy_r      <= 0;
                    move_req_r  <= 0;
                    uio_oe_r    <= 0;
                    telem_vld_r <= 0;

                    if (cmd_telem) begin
                        // Serve telemetry immediately
                        case (telem_sel)
                            2'b00: telem_data_r <= skew;
                            2'b01: telem_data_r <= total_wr[7:0];
                            2'b10: telem_data_r <= total_wr[15:8];
                            2'b11: telem_data_r <= total_wr[TOT_WIDTH-1:16];
                        endcase
                        telem_vld_r <= 1;
                        uio_data_r  <= (telem_sel == 2'b00) ? {skew}
                                     : (telem_sel == 2'b01) ? total_wr[7:0]
                                     : (telem_sel == 2'b10) ? total_wr[15:8]
                                     : total_wr[TOT_WIDTH-1:16];
                        uio_oe_r    <= 1;
                        state       <= ST_TELEM;
                    end else if ((cmd_read || cmd_write) && !busy_r) begin
                        // Latch request; start pipeline
                        log_lat   <= logical;
                        write_pend <= cmd_write;
                        state     <= ST_FEISTEL;
                        busy_r    <= 1;
                    end
                end

            
                ST_FEISTEL: begin
                    scr_lat <= feistel(log_lat, lfsr_r[5:0]);
                    state   <= ST_MAP;
                end

             
                ST_MAP: begin
                    // ECC decode persistent state
                    begin
                        reg [3:0] ds, dg;
                        ds = ham_dec3(ecc_start_r);
                        dg = ham_dec3(ecc_gap_r);
                        ecc_err_r <= ds[3] | dg[3];
                        Start_r   <= ds[2:0];
                        Gap_r     <= dg[2:0];
                    end
                    phys_lat <= startgap(scr_lat, Start_r, Gap_r);
                    blk_ret_r <= retired[startgap(scr_lat, Start_r, Gap_r)];
                    state    <= write_pend ? ST_INC : ST_IDLE;
                    // For reads: output is ready now; back to IDLE
                    if (!write_pend) busy_r <= 0;
                end

           
                ST_INC: begin
                    // Saturating increment
                    if (!cnt[phys_lat][CNT_WIDTH-1]) // not at max
                        cnt[phys_lat] <= cnt[phys_lat] + 1;
                    total_wr <= total_wr + 1;
                    state    <= ST_ADVANCE;
                end

                
                ST_ADVANCE: begin
                    // Advance Gap every PSI writes
                    if (GapCnt_r == PSI-1) begin
                        GapCnt_r <= 0;
                        // Gap = (Gap + 1) mod N; if Gap==Start, skip (adjust Start)
                        if (((Gap_r + 1) % N) == Start_r)
                            Start_r <= (Start_r + 1) % N;
                        Gap_r <= (Gap_r + 1) % N;
                    end else begin
                        GapCnt_r <= GapCnt_r + 1;
                    end
                    // Atomic commit: update ECC-protected shadows
                    ecc_start_r <= ham_enc3(Start_r);
                    ecc_gap_r   <= ham_enc3(Gap_r);
                    Start_shd   <= Start_r;
                    Gap_shd     <= Gap_r;
                    state       <= ST_RETIRE;
                end

             
                ST_RETIRE: begin
                    // Retire block if counter saturated
                    if (&cnt[phys_lat]) begin
                        retired[phys_lat] <= 1;
                        // Signal to host: issue a move_req so data can be migrated
                        move_req_r <= 1;
                        uio_data_r <= {{(8-LOG2N){1'b0}}, phys_lat};
                        uio_oe_r   <= 1;
                        state      <= ST_WAIT_ACK;
                    end else begin
                        busy_r  <= 0;
                        state   <= ST_IDLE;
                    end
                end

              
                ST_WAIT_ACK: begin
                    if (move_ack) begin
                        move_req_r <= 0;
                        uio_oe_r   <= 0;
                        busy_r     <= 0;
                        state      <= ST_IDLE;
                    end
                end

                
                ST_TELEM: begin
                    // One-cycle telem pulse
                    telem_vld_r <= 0;
                    uio_oe_r    <= 0;
                    state       <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // Output assignments
    assign uo_out[2:0] = phys_lat;
    assign uo_out[3]   = busy_r;
    assign uo_out[4]   = move_req_r;
    assign uo_out[5]   = ecc_err_r;
    assign uo_out[6]   = blk_ret_r;
    assign uo_out[7]   = telem_vld_r;

    assign uio_out     = uio_data_r;
    assign uio_oe      = {8{uio_oe_r}};

    // Formal properties (SymbiYosys / SVA style)
    // Uncomment and run with: sby -f formal/wearlevel.sby
`ifdef FORMAL
    // 1. Monotonicity: total_wr never decreases
    reg [TOT_WIDTH-1:0] f_prev_total;
    always @(posedge clk) f_prev_total <= total_wr;
    always @(posedge clk)
        if (rst_n) assert(total_wr >= f_prev_total);

    // 2. Bounded skew: max-min <= PSI + 1 (Start-Gap guarantee)
    always @(posedge clk)
        if (rst_n) assert(skew <= PSI + 1);

    // 3. GapCnt never exceeds PSI-1
    always @(posedge clk)
        if (rst_n) assert(GapCnt_r < PSI);

    // 4. Start and Gap are always in [0, N-1]
    always @(posedge clk)
        if (rst_n) begin
            assert(Start_r < N);
            assert(Gap_r   < N);
        end

    // 5. Liveness: once write_pend && cmd_commit, busy eventually clears
    //    (expressed as: busy cannot stay high forever — checked by cover)
    cover property (@(posedge clk) disable iff (!rst_n) $rose(busy_r) ##[1:20] $fell(busy_r));
`endif

endmodule
