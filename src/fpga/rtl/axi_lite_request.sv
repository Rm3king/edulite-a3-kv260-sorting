`timescale 1ns/1ps
// One outstanding request. AW and W may arrive independently; responses hold
// until accepted. Invalid alignment and byte strobes are handled by the backend.
module axi_lite_request (
    input wire clk, resetn,
    input wire [11:0] s_axi_awaddr, input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid, output wire s_axi_awready,
    input wire [31:0] s_axi_wdata, input wire [3:0] s_axi_wstrb,
    input wire s_axi_wvalid, output wire s_axi_wready,
    output reg [1:0] s_axi_bresp, output reg s_axi_bvalid, input wire s_axi_bready,
    input wire [11:0] s_axi_araddr, input wire [2:0] s_axi_arprot,
    input wire s_axi_arvalid, output wire s_axi_arready,
    output reg [31:0] s_axi_rdata, output reg [1:0] s_axi_rresp,
    output reg s_axi_rvalid, input wire s_axi_rready,
    output reg req_valid, output reg req_write,
    output reg [11:0] req_addr, output reg [31:0] req_data,
    output reg [3:0] req_strb,
    input wire req_done, input wire [31:0] rsp_data, input wire rsp_error
);
    reg aw_full, w_full, ar_full;
    reg [11:0] aw_saved, ar_saved;
    reg [31:0] w_saved;
    reg [3:0] strb_saved;
    assign s_axi_awready = resetn && !aw_full && !s_axi_bvalid;
    assign s_axi_wready = resetn && !w_full && !s_axi_bvalid;
    assign s_axi_arready = resetn && !ar_full && !s_axi_rvalid;
    always @(posedge clk) begin
        if (!resetn) begin
            aw_full<=0; w_full<=0; ar_full<=0; aw_saved<=0; ar_saved<=0;
            w_saved<=0; strb_saved<=0;
            s_axi_bvalid<=0; s_axi_rvalid<=0; s_axi_bresp<=0; s_axi_rresp<=0; s_axi_rdata<=0;
            req_valid<=0; req_write<=0; req_addr<=0; req_data<=0; req_strb<=0;
        end else begin
            if(s_axi_awvalid && s_axi_awready) begin aw_full<=1; aw_saved<=s_axi_awaddr; end
            if(s_axi_wvalid && s_axi_wready) begin w_full<=1; w_saved<=s_axi_wdata; strb_saved<=s_axi_wstrb; end
            if(s_axi_arvalid && s_axi_arready) begin ar_full<=1; ar_saved<=s_axi_araddr; end
            if(s_axi_bvalid && s_axi_bready) s_axi_bvalid<=0;
            if(s_axi_rvalid && s_axi_rready) s_axi_rvalid<=0;
            if(!req_valid) begin
                if(aw_full && w_full && !s_axi_bvalid) begin
                    req_valid<=1; req_write<=1; req_addr<=aw_saved;
                    req_data<=w_saved; req_strb<=strb_saved;
                end else if(ar_full && !s_axi_rvalid) begin
                    req_valid<=1; req_write<=0; req_addr<=ar_saved; req_data<=0; req_strb<=0;
                end
            end else if(req_done) begin
                req_valid<=0;
                if(req_write) begin
                    aw_full<=0; w_full<=0; s_axi_bvalid<=1; s_axi_bresp<=rsp_error ? 2'b10:2'b00;
                end else begin
                    ar_full<=0; s_axi_rvalid<=1; s_axi_rdata<=rsp_data;
                    s_axi_rresp<=rsp_error ? 2'b10:2'b00;
                end
            end
        end
    end
endmodule
