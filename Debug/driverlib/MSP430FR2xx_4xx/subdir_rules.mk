################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
driverlib/MSP430FR2xx_4xx/%.obj: ../driverlib/MSP430FR2xx_4xx/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"/home/david/ti/ccs1281/ccs/tools/compiler/ti-cgt-msp430_21.6.2.LTS/bin/cl430" -vmspx --use_hw_mpy=F5 --include_path="/home/david/ti/ccs1281/ccs/ccs_base/msp430/include" --include_path="/home/david/workspace_v12/si5351Quad" --include_path="/home/david/workspace_v12/si5351Quad/driverlib/MSP430FR2xx_4xx" --include_path="/home/david/ti/ccs1281/ccs/tools/compiler/ti-cgt-msp430_21.6.2.LTS/include" --advice:power="none" --advice:hw_config=all --define=DEPRECATED --define=__MSP430FR2476__ -g --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="driverlib/MSP430FR2xx_4xx/$(basename $(<F)).d_raw" --obj_directory="driverlib/MSP430FR2xx_4xx" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


