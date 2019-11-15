#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-XC16_dsPIC30F4011.mk)" "nbproject/Makefile-local-XC16_dsPIC30F4011.mk"
include nbproject/Makefile-local-XC16_dsPIC30F4011.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=XC16_dsPIC30F4011
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=configuration_bits.c main.c globals.c ISRs.c user_interface.c slow_event.c setup.c medium_event.c uart.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/configuration_bits.o ${OBJECTDIR}/main.o ${OBJECTDIR}/globals.o ${OBJECTDIR}/ISRs.o ${OBJECTDIR}/user_interface.o ${OBJECTDIR}/slow_event.o ${OBJECTDIR}/setup.o ${OBJECTDIR}/medium_event.o ${OBJECTDIR}/uart.o
POSSIBLE_DEPFILES=${OBJECTDIR}/configuration_bits.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/globals.o.d ${OBJECTDIR}/ISRs.o.d ${OBJECTDIR}/user_interface.o.d ${OBJECTDIR}/slow_event.o.d ${OBJECTDIR}/setup.o.d ${OBJECTDIR}/medium_event.o.d ${OBJECTDIR}/uart.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/configuration_bits.o ${OBJECTDIR}/main.o ${OBJECTDIR}/globals.o ${OBJECTDIR}/ISRs.o ${OBJECTDIR}/user_interface.o ${OBJECTDIR}/slow_event.o ${OBJECTDIR}/setup.o ${OBJECTDIR}/medium_event.o ${OBJECTDIR}/uart.o

# Source Files
SOURCEFILES=configuration_bits.c main.c globals.c ISRs.c user_interface.c slow_event.c setup.c medium_event.c uart.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-XC16_dsPIC30F4011.mk dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=30F4011
MP_LINKER_FILE_OPTION=,--script=p30F4011.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/configuration_bits.o: configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  configuration_bits.c  -o ${OBJECTDIR}/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/configuration_bits.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/globals.o: globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/globals.o.d 
	@${RM} ${OBJECTDIR}/globals.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  globals.c  -o ${OBJECTDIR}/globals.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/globals.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/globals.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/ISRs.o: ISRs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ISRs.o.d 
	@${RM} ${OBJECTDIR}/ISRs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ISRs.c  -o ${OBJECTDIR}/ISRs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/ISRs.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/ISRs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/user_interface.o: user_interface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/user_interface.o.d 
	@${RM} ${OBJECTDIR}/user_interface.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  user_interface.c  -o ${OBJECTDIR}/user_interface.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/user_interface.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/user_interface.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/slow_event.o: slow_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/slow_event.o.d 
	@${RM} ${OBJECTDIR}/slow_event.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  slow_event.c  -o ${OBJECTDIR}/slow_event.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/slow_event.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/slow_event.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/setup.o: setup.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/setup.o.d 
	@${RM} ${OBJECTDIR}/setup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  setup.c  -o ${OBJECTDIR}/setup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/setup.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/setup.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/medium_event.o: medium_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/medium_event.o.d 
	@${RM} ${OBJECTDIR}/medium_event.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  medium_event.c  -o ${OBJECTDIR}/medium_event.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/medium_event.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/medium_event.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/uart.o: uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c  -o ${OBJECTDIR}/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/uart.o.d"      -g -D__DEBUG     -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/configuration_bits.o: configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  configuration_bits.c  -o ${OBJECTDIR}/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/configuration_bits.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/globals.o: globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/globals.o.d 
	@${RM} ${OBJECTDIR}/globals.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  globals.c  -o ${OBJECTDIR}/globals.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/globals.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/globals.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/ISRs.o: ISRs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ISRs.o.d 
	@${RM} ${OBJECTDIR}/ISRs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ISRs.c  -o ${OBJECTDIR}/ISRs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/ISRs.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/ISRs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/user_interface.o: user_interface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/user_interface.o.d 
	@${RM} ${OBJECTDIR}/user_interface.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  user_interface.c  -o ${OBJECTDIR}/user_interface.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/user_interface.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/user_interface.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/slow_event.o: slow_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/slow_event.o.d 
	@${RM} ${OBJECTDIR}/slow_event.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  slow_event.c  -o ${OBJECTDIR}/slow_event.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/slow_event.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/slow_event.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/setup.o: setup.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/setup.o.d 
	@${RM} ${OBJECTDIR}/setup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  setup.c  -o ${OBJECTDIR}/setup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/setup.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/setup.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/medium_event.o: medium_event.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/medium_event.o.d 
	@${RM} ${OBJECTDIR}/medium_event.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  medium_event.c  -o ${OBJECTDIR}/medium_event.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/medium_event.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/medium_event.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/uart.o: uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/uart.o.d 
	@${RM} ${OBJECTDIR}/uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  uart.c  -o ${OBJECTDIR}/uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/uart.o.d"        -g -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x84F   -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_XC16_dsPIC30F4011=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/AN901.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/XC16_dsPIC30F4011
	${RM} -r dist/XC16_dsPIC30F4011

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
