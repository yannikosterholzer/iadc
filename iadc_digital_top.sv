
module iadc_digital_top #(
    // Analog Control FSM parameters
    parameter int unsigned N_C                 = 2069,
    parameter int unsigned N_RU                = 1024,
    parameter int unsigned N_DE_MAX            = 1024,
    parameter int unsigned T_R                 = 8,
    parameter int unsigned T_S                 = 3,
    // Error Handler parameters 
    parameter int unsigned BUF_LEN             = 4,
    parameter int unsigned DELTA_SHIFT         = 2,
    parameter int          CRITICAL_RANGE      = 100,
    parameter int          OUTLIER_THRESH      = 128,
    parameter int          CLIP_VAL            = 1023,
    parameter int unsigned PIPE_LATENCY        = 2
) (
    input  logic       clk,
    input  logic       resetn,         // active-low async reset
    input  logic       pwrdn,          // pin kept for interface compat; unused internally
    input  logic       vcomp,          // comparator output
    output logic       s1,
    output logic       s2a,
    output logic       s2b,
    output logic       s3,             // also discharges integration cap during REC_FIX
    output logic [9:0] dout,           // corrected magnitude
    output logic       dout_sgn,       // corrected sign bit
    output logic       clk_out,        // valid-pulse for output (one clock per conversion)
    output logic       flag_estimated  // 1 = corrected, 0 = measured
);

    logic   res_n;
    assign  res_n = !pwrdn | resetn;

    logic       int_dout_valid;     // analog_ctrl -> error_handler.sample_pulse
    logic [9:0] int_cnt_out;        // raw counter magnitude
    logic       int_sgn_out;        // raw sign
    logic       int_ovf_out;        // raw overflow flag


    analog_ctrl #(
        .N_C      (N_C),
        .N_RU     (N_RU),
        .N_DE_MAX (N_DE_MAX),
        .T_R      (T_R),
        .T_S      (T_S)
    ) u_analog_ctrl (
        .clk          (clk),
        .resetn       (res_n),
        .vcomp        (vcomp),

        .s1           (s1),
        .s2a          (s2a),
        .s2b          (s2b),
        .s3           (s3),

        .dout_valid   (int_dout_valid),
        .cnt_out      (int_cnt_out),
        .sgn_out      (int_sgn_out),
        .ovf_out      (int_ovf_out)
    );


    error_handler #(
        .BUF_LEN         (BUF_LEN),
        .DELTA_SHIFT     (DELTA_SHIFT),
        .CRITICAL_RANGE  (CRITICAL_RANGE),
        .OUTLIER_THRESH  (OUTLIER_THRESH),
        .CLIP_VAL        (CLIP_VAL),
        .PIPE_LATENCY    (PIPE_LATENCY)
    ) u_error_handler (
        .clk            (clk),
        .resetn         (res_n),

        .sample_pulse   (int_dout_valid),
        .cnt_in         (int_cnt_out),
        .sgn_in         (int_sgn_out),
        .ovf_in         (int_ovf_out),

        .clk_out        (clk_out),
        .dout           (dout),
        .dout_sgn       (dout_sgn),
        .flag_estimated (flag_estimated)
    );

endmodule
