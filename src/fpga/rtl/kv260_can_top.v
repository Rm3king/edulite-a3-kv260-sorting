`timescale 1ns/1ps
module kv260_can_top (

    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET resetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    input wire clk,
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 resetn RST" *)
    input wire resetn,
    input wire [11:0] s_axi_awaddr, input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid, output wire s_axi_awready,
    input wire [31:0] s_axi_wdata, input wire [3:0] s_axi_wstrb,
    input wire s_axi_wvalid, output wire s_axi_wready,
    output wire [1:0] s_axi_bresp, output wire s_axi_bvalid, input wire s_axi_bready,
    input wire [11:0] s_axi_araddr, input wire [2:0] s_axi_arprot,
    input wire s_axi_arvalid, output wire s_axi_arready,
    output wire [31:0] s_axi_rdata, output wire [1:0] s_axi_rresp,
    output wire s_axi_rvalid, input wire s_axi_rready,
    input wire can_rx, output wire can_tx,
    output wire [31:0] debug_us, output wire [7:0] debug_status
);
 wire req_valid,req_write,req_done,rsp_error;
 wire [11:0] req_addr; wire [31:0] req_data,rsp_data; wire [3:0] req_strb;
 axi_lite_request axi(.clk(clk),.resetn(resetn),
    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awprot(s_axi_awprot),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_araddr(s_axi_araddr),
    .s_axi_arprot(s_axi_arprot),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready),
    .req_valid(req_valid),.req_write(req_write),.req_addr(req_addr),.req_data(req_data),
    .req_strb(req_strb),.req_done(req_done),.rsp_data(rsp_data),.rsp_error(rsp_error));
 can_engine engine(.clk(clk),.resetn(resetn),.can_rx(can_rx),.can_tx(can_tx),
    .req_valid(req_valid),.req_write(req_write),.req_addr(req_addr),.req_data(req_data),
    .req_strb(req_strb),.req_done(req_done),.rsp_data(rsp_data),.rsp_error(rsp_error),
    .debug_us(debug_us),.debug_status(debug_status));
endmodule
