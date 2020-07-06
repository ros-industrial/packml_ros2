# PackML State, based on the PackML standard, summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf

# Enumeration values are based upon tag guidelines, here:
# http://sesam-world.com/_pdf/make2pack/mode/2010-11-29/Materiale/PackML_Tag_Naming_Guidelines_V2.0.pdf
# Due to changes in the standard some states weren't enumerated.  I've made my best guess
# below

# Values defined in tag naming guidelines
int8 UNDEFINED = 0
int8 OFF = 1            # Deleted in V3.0 of packml
int8 STOPPED = 2
int8 STARTING = 3
int8 IDLE = 4
int8 SUSPENDED = 5
int8 EXECUTE = 6
int8 STOPPING = 7
int8 ABORTING = 8
int8 ABORTED = 9
int8 HOLDING = 10
int8 HELD = 11

# Values NOT defined in tag naming guidelines
int8 RESETTING = 100
int8 SUSPENDING = 101    # Not enumerated in tag guidelines
int8 UNSUSPENDING = 102
int8 CLEARING = 103
int8 UNHOLDING = 104
int8 COMPLETING = 105
int8 COMPLETE = 106

int8 val
