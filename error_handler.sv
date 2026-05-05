module error_handler #(
    parameter int unsigned BUF_LEN              = 4,    // FIFO depth
    parameter int unsigned DELTA_SHIFT          = 2,    // log2 of delta-divisor
    parameter int          CRITICAL_RANGE       = 100,  // |expected| < this => critical zone
    parameter int          OUTLIER_THRESH       = 128,  // |v_meas - expected| > this => outlier
    parameter int          CLIP_VAL             = 1023, // value substituted on input overflow
    parameter int unsigned PIPE_LATENCY         = 2     // clocks from sample_pulse to clk_out
) (
    input  logic                clk,
    input  logic                resetn,
    input  logic                sample_pulse,    // level pulse marking valid input window
    input  logic  [9:0]         cnt_in,          // deint counter magnitude
    input  logic                sgn_in,          // sign bit (0 = pos, 1 = neg)
    input  logic                ovf_in,          // input overflow flag
    output logic                clk_out,      // valid pulse delayed by PIPE_LATENCY
    output logic  [9:0]         dout,            // corrected magnitude
    output logic                dout_sgn,        // corrected sign bit
    output logic                flag_estimated   // 1 = value was corrected, 0 = measured
);

    typedef logic signed [11:0] sval_t;          // internal 12-bit signed datum

    typedef enum logic {
        MEASURED  = 1'b0,
        ESTIMATED = 1'b1
    } flag_e;

    typedef struct packed {
        sval_t  value;
        flag_e  flag;
    } buf_entry_t;


    typedef enum logic [2:0] {
        S_IDLE_WARM   = 3'b000,
        S_CALC_WARM   = 3'b010,
        S_COMMIT_WARM = 3'b100,
        S_HOLD_WARM   = 3'b110,
        S_IDLE_RUN    = 3'b001,
        S_CALC_RUN    = 3'b011,
        S_COMMIT_RUN  = 3'b101,
        S_HOLD_RUN    = 3'b111
    } state_e;

    state_e  state_q, state_d;

    always_ff @(posedge clk or negedge resetn) begin : FSM_REG
        if (!resetn)    state_q <= S_IDLE_WARM;
        else            state_q <= state_d;
    end

    // Mode flag derived from state encoding: 0 = warmup, 1 = running.
    logic  mode_run;
    assign mode_run = state_q[0];


    logic  calc_en;
    assign calc_en = (state_q == S_IDLE_WARM && sample_pulse) ||
                     (state_q == S_IDLE_RUN  && sample_pulse);

    logic [PIPE_LATENCY-1:0]  pulse_dly_q;

    always_ff @(posedge clk or negedge resetn) begin : PULSE_DELAY
        if (!resetn)    pulse_dly_q <= '0;
        else            pulse_dly_q <= {pulse_dly_q[PIPE_LATENCY-2:0], calc_en};
    end

    assign clk_out = pulse_dly_q[PIPE_LATENCY-1];


    buf_entry_t  buffer_q  [0:BUF_LEN-1];
    buf_entry_t  buffer_d  [0:BUF_LEN-1];

    logic  lockout_c;

    assign lockout_c = (buffer_q[BUF_LEN-1].flag == ESTIMATED) &&
                       (buffer_q[BUF_LEN-2].flag == ESTIMATED) &&
                       (buffer_q[BUF_LEN-3].flag == ESTIMATED);

    logic [9:0]  x_mag_c;
    logic        x_sign_c;
    sval_t       x_ext_c;
    sval_t       v_meas_c;

    always_comb begin : INPUT_DECODE
        x_mag_c  = ovf_in ? 10'(CLIP_VAL) : cnt_in;
        x_sign_c = ovf_in ? buffer_q[BUF_LEN-1].value[11] : sgn_in;
        x_ext_c  = sval_t'({2'b00, x_mag_c});
        v_meas_c = x_sign_c ? -x_ext_c : x_ext_c;
    end

    logic signed [12:0]  delta_c;
    sval_t               delta_div_c;
    sval_t               expected_c;

    always_comb begin : PREDICT
        delta_c   = $signed({buffer_q[BUF_LEN-1].value[11], buffer_q[BUF_LEN-1].value})
                  - $signed({buffer_q[0].value[11],          buffer_q[0].value});
        delta_div_c = sval_t'(delta_c >>> DELTA_SHIFT);
        expected_c  = buffer_q[BUF_LEN-1].value + delta_div_c;
    end

    sval_t  diff_c;
    sval_t  abs_diff_c;
    logic   in_critical_c;
    logic   is_outlier_c;
    logic   use_expected_c;

    always_comb begin : ZONE_DECISION
        in_critical_c = (expected_c > -sval_t'(CRITICAL_RANGE)) &&
                        (expected_c <  sval_t'(CRITICAL_RANGE));
        diff_c        = v_meas_c - expected_c;
        abs_diff_c    = diff_c[11] ? -diff_c : diff_c;
        is_outlier_c  = (abs_diff_c > sval_t'(OUTLIER_THRESH));

        // mode_run gates the prediction: during warmup, always pass v_meas.
        use_expected_c = mode_run && in_critical_c && (is_outlier_c || ovf_in);
    end

    sval_t  out_val_c;
    flag_e  out_flag_c;

    always_comb begin : FINAL_MUX
        if (use_expected_c) begin
            out_val_c  = expected_c;
            out_flag_c = ESTIMATED;
        end
        else begin
            out_val_c  = v_meas_c;
            out_flag_c = MEASURED;
        end
    end


    logic  commit_now_c;
    assign commit_now_c = (state_q == S_COMMIT_WARM) || (state_q == S_COMMIT_RUN);

    buf_entry_t  new_entry_c;

    always_comb begin : NEW_ENTRY_BUILD
        new_entry_c.value = out_val_c;
        new_entry_c.flag  = out_flag_c;
    end

    always_comb begin : BUF_NEXT
        // Default: hold current values.
        for (int i = 0; i < BUF_LEN; i++) begin
            buffer_d[i] = buffer_q[i];
        end

        if (commit_now_c) begin
            // Shift left, insert new entry at top.
            for (int i = 0; i < BUF_LEN-1; i++) begin
                buffer_d[i] = buffer_q[i+1];
            end
            buffer_d[BUF_LEN-1] = new_entry_c;
            if ((state_q == S_COMMIT_RUN) && lockout_c) begin
                for (int i = 0; i < BUF_LEN; i++) begin
                    buffer_d[i].flag = ESTIMATED;
                end
            end
        end
    end

    always_ff @(posedge clk or negedge resetn) begin : BUF_REG
        if (!resetn) begin
            for (int i = 0; i < BUF_LEN; i++) begin
                buffer_q[i].value <= '0;
                buffer_q[i].flag  <= ESTIMATED;
            end
        end else begin
            buffer_q <= buffer_d;
        end
    end

    logic  warmup_done_c;

    assign warmup_done_c = ~(buffer_d[0].flag | buffer_d[1].flag |
                             buffer_d[2].flag | buffer_d[3].flag);

    always_comb begin : FSM_NEXT
        state_d = state_q;
        unique case (state_q)
            S_IDLE_WARM:    state_d = sample_pulse  ? S_CALC_WARM   : S_IDLE_WARM;
            S_CALC_WARM:    state_d = S_COMMIT_WARM;
            S_COMMIT_WARM:  state_d = warmup_done_c ? S_HOLD_RUN    : S_HOLD_WARM;
            S_HOLD_WARM:    state_d = sample_pulse  ? S_HOLD_WARM   : S_IDLE_WARM;
            S_IDLE_RUN:     state_d = sample_pulse  ? S_CALC_RUN    : S_IDLE_RUN;
            S_CALC_RUN:     state_d = S_COMMIT_RUN;
            S_COMMIT_RUN:   state_d = lockout_c     ? S_HOLD_WARM   : S_HOLD_RUN;
            S_HOLD_RUN:     state_d = sample_pulse  ? S_HOLD_RUN    : S_IDLE_RUN;
            default:        state_d = S_IDLE_WARM;
        endcase
    end

    sval_t       out_abs_c;
    logic [9:0]  out_mag_c;
    logic        out_sgn_c;

    always_comb begin : OUTPUT_FORMAT
        out_sgn_c = out_val_c[11];
        out_abs_c = out_val_c[11] ? -out_val_c : out_val_c;
        if (out_abs_c > sval_t'(CLIP_VAL))  out_mag_c = 10'(CLIP_VAL);
        else                                out_mag_c = out_abs_c[9:0];
    end

    always_ff @(posedge clk or negedge resetn) begin : OUTPUT_REG
        if (!resetn) begin
            dout           <= '0;
            dout_sgn       <= 1'b0;
            flag_estimated <= 1'b0;
        end
        else if (commit_now_c) begin
            dout           <= out_mag_c;
            dout_sgn       <= out_sgn_c;
            flag_estimated <= (out_flag_c == ESTIMATED);
        end
    end

endmodule
