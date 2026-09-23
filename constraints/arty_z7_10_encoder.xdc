## arty_z7_10_encoder.xdc
## Pin constraints for axi_quad_encoder block design, Arty Z7-10
## Pins verified against Digilent's official Arty-Z7-10-Master.xdc

## Encoder inputs -- Pmod JA (single-ended use of differential-pair pins;
## Digilent's own reference manual notes this is fine, just watch for
## crosstalk between the paired P/N signals if using both halves)
set_property -dict { PACKAGE_PIN Y18  IOSTANDARD LVCMOS33 } [get_ports { A_pin_0 }];           # JA1_P
set_property -dict { PACKAGE_PIN Y16  IOSTANDARD LVCMOS33 } [get_ports { B_pin_0 }];           # JA2_P
set_property -dict { PACKAGE_PIN U18  IOSTANDARD LVCMOS33 } [get_ports { Z_pin_0 }];           # JA3_P
set_property -dict { PACKAGE_PIN W18  IOSTANDARD LVCMOS33 } [get_ports { fault_clear_ext_0 }]; # JA4_P

## Status LEDs (simple single-color LEDs LD0/LD1, not the RGB LD4/LD5)
set_property -dict { PACKAGE_PIN R14  IOSTANDARD LVCMOS33 } [get_ports { led_fault_0 }];       # LD0
set_property -dict { PACKAGE_PIN P14  IOSTANDARD LVCMOS33 } [get_ports { led_direction_0 }];   # LD1
