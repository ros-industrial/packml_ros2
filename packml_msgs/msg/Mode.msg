# PackML Mode, based on the PackML standard, summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf

# Enumeration values are based upon tag guidelines, here:
# http://sesam-world.com/_pdf/make2pack/mode/2010-11-29/Materiale/PackML_Tag_Naming_Guidelines_V2.0.pdf

int8 UNDEFINED = 0
int8 AUTOMATIC = 1
int8 SEMI_AUTOMATIC = 2
int8 MANUAL = 3
int8 IDLE = 4
int8 SETUP = 11  # not defined in tag standard, but mentioned in state standard

int8 val
