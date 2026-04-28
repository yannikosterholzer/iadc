module error_handler #(
    parameter int unsigned BUF_LEN              = 5,    // 5 entries
    parameter int unsigned DELTA_HIST           = 4,    // must be power of 2
    parameter int unsigned DELTA_SHIFT          = 2,    // log2(DELTA_HIST)
    parameter int unsigned WARMUP_SAMPLES       = 4,
    parameter int          CRITICAL_RANGE       = 100,
    parameter int          FULL_SCALE_HI        = 900,
    parameter int          CLIP_VAL             = 1023,
    parameter int          OUTLIER_THRESH_CRIT  = 100,
    parameter int          OUTLIER_THRESH_NORM  = 150,
    parameter int unsigned PIPE_LATENCY         = 3     // clocks from pulse-in to pulse-out
) (
    input  logic                clk,
    input  logic                resetn,          
    input  logic                pwrdn,           // synchronous power-down (already synchronized in toplevel)
    input  logic                sample_pulse,    // level pulse marking valid input window
    input  logic  [9:0]         cnt_in,          // deint counter magnitude
    input  logic                sgn_in,          // sign bit (0=pos, 1=neg)
    input  logic                ovf_in,          // overflow flag
    output logic                dout_pulse,      // dout valid-pulse delayed by PIPE_LATENCY
    output logic  [9:0]         dout,            // corrected magnitude
    output logic                dout_sgn,        // corrected sign bit
    output logic                flag_estimated   // 1 = value corrected, 0 = measured
);

    typedef enum logic {
        MEASURED  = 1'b0,
        ESTIMATED = 1'b1
    } flag_e;

    typedef enum logic {
        WARMUP  = 1'b0,
        RUNNING = 1'b1
    } warmup_state_e;

    typedef logic signed [11:0] sval_t;

    typedef struct packed {
        sval_t  value;
        flag_e  flag;
    } buf_entry_t;

    typedef enum logic [1:0] {
        SEQ_IDLE  = 2'b00,
        SEQ_BUSY  = 2'b01,
        SEQ_HOLD  = 2'b10
    } seq_state_e;

    seq_state_e  seq_state, seq_next;
    logic        seq_start;

    always_ff @(posedge clk or negedge resetn) begin : SEQ_REG
        if (!resetn)     seq_state <= SEQ_IDLE;
        else if (pwrdn)  seq_state <= SEQ_IDLE;
        else             seq_state <= seq_next;
    end

    always_comb begin : SEQ_TRANS
        seq_next  = seq_state;
        seq_start = 1'b0;
        unique case (seq_state)
            SEQ_IDLE: begin
                if (sample_pulse) begin
                    seq_start = 1'b1;
                    seq_next  = SEQ_BUSY;
                end
            end
            SEQ_BUSY: begin
                seq_next = SEQ_HOLD;
            end
            SEQ_HOLD: begin
                if (!sample_pulse)
                    seq_next = SEQ_IDLE;
            end
            default: seq_next = SEQ_IDLE;
        endcase
    end

    logic [PIPE_LATENCY-1:0]  pulse_dly;

    always_ff @(posedge clk or negedge resetn) begin : PULSE_DELAY
        if (!resetn)     pulse_dly <= '0;
        else if (pwrdn)  pulse_dly <= '0;
        else             pulse_dly <= {pulse_dly[PIPE_LATENCY-2:0], sample_pulse};
    end

    assign dout_pulse = pulse_dly[PIPE_LATENCY-1];

    function automatic sval_t abs_s (input sval_t x);
        abs_s = (x < 0) ? -x : x;
    endfunction

    sval_t v_meas;

    always_comb begin : INPUT_DECODE
        sval_t cnt_ext;
        cnt_ext = sval_t'({2'b00, cnt_in});
        if (ovf_in)        v_meas = '0;
        else if (sgn_in)   v_meas = -cnt_ext;
        else               v_meas =  cnt_ext;
    end

    buf_entry_t  buffer    [0:BUF_LEN-1];
    buf_entry_t  new_entry;

    logic lockout_c; // Detects predictor runaway

    assign lockout_c = logic'(buffer[BUF_LEN-1].flag) & logic'(buffer[BUF_LEN-2].flag) & logic'(buffer[BUF_LEN-3].flag);

    warmup_state_e                       wm_state, wm_next;
    logic [$clog2(WARMUP_SAMPLES+1):0]   sample_count;

    always_ff @(posedge clk or negedge resetn) begin : WM_CNT
        if (!resetn)
            sample_count <= '0;
        else if (pwrdn)
            sample_count <= '0;
        else if (seq_start) begin
            // Reset count on lockout so re-warmup starts fresh
            if (wm_state == RUNNING && lockout_c)
                sample_count <= '0;
            else if (wm_state == WARMUP)
                sample_count <= sample_count + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge resetn) begin : WM_STATE_REG
        if (!resetn)         wm_state <= WARMUP;
        else if (pwrdn)      wm_state <= WARMUP;
        else if (seq_start)  wm_state <= wm_next;
    end

    always_comb begin : WM_TRANS
        wm_next = wm_state;
        unique case (wm_state)
            WARMUP:  if (sample_count >= WARMUP_SAMPLES[$bits(sample_count)-1:0])
                         wm_next = RUNNING;
            RUNNING: if (lockout_c)
                         wm_next = WARMUP;        // soft re-warmup on predictor runaway
                     else
                         wm_next = RUNNING;
        endcase
    end

    logic signed [13:0]  delta_acc_c;
    sval_t               avg_delta_c;
    sval_t               v_est_c;
    sval_t               v_last_c;

    always_comb begin : CALC_DELTA_AVG
        delta_acc_c = '0;
        delta_acc_c = signed'({{2{buffer[BUF_LEN-1].value[11]}}, buffer[BUF_LEN-1].value}) - 
                      signed'({{2{buffer[0].value[11]}}, buffer[0].value});
        
        avg_delta_c = sval_t'(delta_acc_c >>> DELTA_SHIFT);
    end

    always_comb begin : CALC_PREDICTION
        v_last_c = buffer[BUF_LEN-1].value;
        v_est_c = sval_t'(signed'({{2{v_last_c[11]}}, v_last_c}) + signed'({{2{avg_delta_c[11]}}, avg_delta_c}));
    end

    sval_t          v_est_s2, v_last_s2, v_meas_s2;
    logic           ovf_s2;
    warmup_state_e  wm_state_s2;
    logic           valid_s2;

    always_ff @(posedge clk or negedge resetn) begin : STAGE1_REG
        if (!resetn) begin
            v_est_s2    <= '0;
            v_last_s2   <= '0;
            v_meas_s2   <= '0;
            ovf_s2      <= 1'b0;
            wm_state_s2 <= WARMUP;
            valid_s2    <= 1'b0;
        end
        else if (pwrdn) begin
            v_est_s2    <= '0;
            v_last_s2   <= '0;
            v_meas_s2   <= '0;
            ovf_s2      <= 1'b0;
            wm_state_s2 <= WARMUP;
            valid_s2    <= 1'b0;
        end
        else begin
            valid_s2 <= seq_start;
            if (seq_start) begin
                v_est_s2    <= v_est_c;
                v_last_s2   <= v_last_c;
                v_meas_s2   <= v_meas;
                ovf_s2      <= ovf_in;
                wm_state_s2 <= (wm_state == RUNNING && lockout_c) ? WARMUP : wm_state;
            end
        end
    end

    sval_t   abs_v_est_s2, abs_v_diff_s2,  v_diff_s2;
    logic    in_critical_s2, in_fullscale_s2, is_outlier_s2;
    sval_t   out_val_c;
    flag_e   out_flag_c;


    always_comb begin : ZONE_DECISION
        abs_v_est_s2   = abs_s(v_est_s2);
        v_diff_s2      = v_meas_s2 - v_est_s2;
        abs_v_diff_s2  = abs_s(v_diff_s2);
        
        in_critical_s2  = (abs_v_est_s2  < sval_t'(CRITICAL_RANGE));
        in_fullscale_s2 = (abs_v_est_s2 >= sval_t'(FULL_SCALE_HI));
        is_outlier_s2   = (abs_v_diff_s2 > sval_t'(OUTLIER_THRESH_NORM));
    end

    always_comb begin : DECISION_LOGIC
        out_val_c  = v_meas_s2;
        out_flag_c = MEASURED;
        if (wm_state_s2 == WARMUP) begin
				if (ovf_s2 && in_fullscale_s2) begin
                    out_val_c  = (v_last_s2 < 0) ? -sval_t'(CLIP_VAL)
                                                 :  sval_t'(CLIP_VAL);
                    out_flag_c = MEASURED;
                end else begin
					out_val_c  = v_meas_s2;
					out_flag_c = MEASURED;
				end
        end
        else begin
			if(in_critical_s2) begin
				if (is_outlier_s2 || ovf_s2) begin
					out_val_c  = v_est_s2;
					out_flag_c = ESTIMATED;
				end
				else begin
					out_val_c  = v_meas_s2;
					out_flag_c = MEASURED;
				end
			end else begin
				if (ovf_s2 && in_fullscale_s2) begin
                    out_val_c  = (v_last_s2 < 0) ? -sval_t'(CLIP_VAL)
                                                 :  sval_t'(CLIP_VAL);
                    out_flag_c = MEASURED;
                end
                else begin
					out_val_c  = v_meas_s2;
					out_flag_c = MEASURED;
                end
            end
		end
	end

    always_comb begin : BUF_PACK
        new_entry.value = out_val_c;
        new_entry.flag  = out_flag_c;
    end

    always_ff @(posedge clk or negedge resetn) begin : BUF_SHIFT
        if (!resetn) begin
            for (int i = 0; i < BUF_LEN; i++) begin
                buffer[i].value <= '0;
                buffer[i].flag  <= MEASURED;
            end
        end
        else if (pwrdn) begin
            for (int i = 0; i < BUF_LEN; i++) begin
                buffer[i].value <= '0;
                buffer[i].flag  <= MEASURED;
            end
        end
        else if (valid_s2) begin
            for (int i = 0; i < BUF_LEN-1; i++) begin
                buffer[i] <= buffer[i+1];
            end
            buffer[BUF_LEN-1] <= new_entry;
        end
    end

    sval_t   out_val_s3;
    flag_e   out_flag_s3;

    always_ff @(posedge clk or negedge resetn) begin : STAGE2_REG
        if (!resetn) begin
            out_val_s3  <= '0;
            out_flag_s3 <= MEASURED;
        end
        else if (pwrdn) begin
            out_val_s3  <= '0;
            out_flag_s3 <= MEASURED;
        end
        else if (valid_s2) begin
            out_val_s3  <= out_val_c;
            out_flag_s3 <= out_flag_c;
        end
    end

    logic [9:0]  dout_c;
    logic        dout_sgn_c;
    logic        flag_estimated_c;

    always_comb begin : OUTPUT_CONVERT
        dout_sgn_c       = out_val_s3[11];
        dout_c           = (out_val_s3 < 0)? 10'(-out_val_s3) : 10'(out_val_s3);
        flag_estimated_c = (out_flag_s3 == ESTIMATED);
    end

    always_ff @(posedge clk or negedge resetn) begin : OUTPUT_REG
        if (!resetn) begin
            dout           <= '0;
            dout_sgn       <= 1'b0;
            flag_estimated <= 1'b0;
        end
        else if (pwrdn) begin
            dout           <= '0;
            dout_sgn       <= 1'b0;
            flag_estimated <= 1'b0;
        end
        else begin
            dout           <= dout_c;
            dout_sgn       <= dout_sgn_c;
            flag_estimated <= flag_estimated_c;
        end
    end

endmodule