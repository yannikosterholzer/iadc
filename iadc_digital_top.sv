module iadc_digital_top #(
    parameter int unsigned N_C                 = 2069,
    parameter int unsigned N_RU                = 1024,
    parameter int unsigned N_DE_MAX            = 1024,
    parameter int unsigned T_R                 = 8,
    parameter int unsigned T_S                 = 3,
    parameter int unsigned BUF_LEN             = 5,
    parameter int unsigned DELTA_HIST          = 4,
    parameter int unsigned DELTA_SHIFT         = 2,
    parameter int unsigned WARMUP_SAMPLES      = 4,
    parameter int          CRITICAL_RANGE      = 50,
    parameter int          FULL_SCALE_HI       = 900,
    parameter int          CLIP_VAL            = 1023,
    parameter int          OUTLIER_THRESH_CRIT = 100,
    parameter int          OUTLIER_THRESH_NORM = 150,
    parameter int unsigned PIPE_LATENCY        = 3
) (
    input  logic       clk,
    input  logic       resetn,         // active-low async reset
    input  logic       pwrdn,          // async powerdown pin
    input  logic       vcomp,          // comparator output
    output logic       s1,
    output logic       s2a,
    output logic       s2b,
    output logic       s3,             // discharges integration cap
    output logic [9:0] dout,           // (corrected) magnitude
    output logic       dout_sgn,       // (corrected) sign bit
    output logic       clk_out,        // valid-signal for output
    output logic       flag_estimated  // 1 = corrected, 0 = measured
);

    logic pwrdn_meta;     // synchronize pwrdn to avoid metastability 
    logic pwrdn_sync;     

    always_ff @(posedge clk or negedge resetn) begin : PWRDN_SYNC
        if (!resetn) begin
            pwrdn_meta <= 1'b0;
            pwrdn_sync <= 1'b0;
        end
        else begin
            pwrdn_meta <= pwrdn;
            pwrdn_sync <= pwrdn_meta;
        end
    end

    logic vcomp_sync1;

    always_ff @(posedge clk or negedge resetn) begin : VCOMP_SYNC
    	if (!resetn) begin
    	    vcomp_sync1 <= 1'b0;
    	end
    	else if (pwrdn) begin
    	    vcomp_sync1 <= 1'b0;
    	end
    	else begin
    	    vcomp_sync1 <= vcomp;
    	end
     end

    logic       valid_fsm2handler;
    logic [9:0] int_cnt_out;
    logic       int_sgn_out;
    logic       int_ovf_out;

    analog_ctrl #(
        .N_C      (N_C),
        .N_RU     (N_RU),
        .N_DE_MAX (N_DE_MAX),
        .T_R      (T_R),
        .T_S      (T_S)
    ) u_analog_ctrl (
        .clk          (clk),
        .resetn       (resetn),
        .pwrdn        (pwrdn_sync),     // synchronized
        .vcomp        (vcomp_sync1),

        .s1           (s1),
        .s2a          (s2a),
        .s2b          (s2b),
        .s3           (s3),

        .dout_valid   (valid_fsm2handler),
        .cnt_out      (int_cnt_out),
        .sgn_out      (int_sgn_out),
        .ovf_out      (int_ovf_out)
    );

    error_handler #(
        .BUF_LEN             (BUF_LEN),
        .DELTA_HIST          (DELTA_HIST),
        .DELTA_SHIFT         (DELTA_SHIFT),
        .WARMUP_SAMPLES      (WARMUP_SAMPLES),
        .CRITICAL_RANGE      (CRITICAL_RANGE),
        .FULL_SCALE_HI       (FULL_SCALE_HI),
        .CLIP_VAL            (CLIP_VAL),
        .OUTLIER_THRESH_CRIT (OUTLIER_THRESH_CRIT),
        .OUTLIER_THRESH_NORM (OUTLIER_THRESH_NORM),
        .PIPE_LATENCY        (PIPE_LATENCY)
    ) u_error_handler (
        .clk            (clk),
        .resetn         (resetn),
        .pwrdn          (pwrdn_sync),    // synchronized

        .sample_pulse   (valid_fsm2handler),
        .cnt_in         (int_cnt_out),
        .sgn_in         (int_sgn_out),
        .ovf_in         (int_ovf_out),

        .dout_pulse     (clk_out),
        .dout           (dout),
        .dout_sgn       (dout_sgn),
        .flag_estimated (flag_estimated)
    );

endmodule
