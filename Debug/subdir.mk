################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Main_ByLee.c 

OBJS += \
./Main_ByLee.obj 

C_DEPS += \
./Main_ByLee.pp 

OBJS__QTD += \
".\Main_ByLee.obj" 

C_DEPS__QTD += \
".\Main_ByLee.pp" 

C_SRCS_QUOTED += \
"../Main_ByLee.c" 


# Each subdirectory must supply rules for building sources it contributes
Main_ByLee.obj: ../Main_ByLee.c $(GEN_OPTS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"D:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="D:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Users/ACIP-Lee/Documents/workspace/AD_ByLee/DSP281x_common/include" --include_path="C:/Users/ACIP-Lee/Documents/workspace/AD_ByLee/DSP281x_headers/include" --include_path="C:/Users/ACIP-Lee/Documents/workspace/AD_ByLee/include" --quiet --diag_warning=225 --large_memory_model --unified_memory --preproc_with_compile --preproc_dependency="Main_ByLee.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '


