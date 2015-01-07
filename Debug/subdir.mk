################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DVR_Main_ByLee.c 

OBJS += \
./DVR_Main_ByLee.obj 

C_DEPS += \
./DVR_Main_ByLee.pp 

OBJS__QTD += \
".\DVR_Main_ByLee.obj" 

C_DEPS__QTD += \
".\DVR_Main_ByLee.pp" 

C_SRCS_QUOTED += \
"../DVR_Main_ByLee.c" 


# Each subdirectory must supply rules for building sources it contributes
DVR_Main_ByLee.obj: ../DVR_Main_ByLee.c $(GEN_OPTS)
	@echo 'Building file: $<'
	@echo 'Invoking: Compiler'
	"D:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/bin/cl2000" --silicon_version=28 -g --define="_DEBUG" --define="LARGE_MODEL" --include_path="D:/Program Files/Texas Instruments/ccsv4/tools/compiler/c2000/include" --include_path="C:/Users/ACIP-Lee/Documents/workspace/DVR_ByLee/include" --diag_warning=225 --large_memory_model --unified_memory --preproc_with_compile --preproc_dependency="DVR_Main_ByLee.pp" $(GEN_OPTS_QUOTED) $(subst #,$(wildcard $(subst $(SPACE),\$(SPACE),$<)),"#")
	@echo 'Finished building: $<'
	@echo ' '


