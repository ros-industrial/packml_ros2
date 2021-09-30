# This PackML status message captures the PackML state machine data.  It's
# meant to be published continuously or latched/on changes (depending on
# implementation)
# Based on PackML summarized here:
# http://www.plcopen.org/pages/promotion/publications/downloads/mapping_omac_statediagram.pdf


builtin_interfaces/Time stamp 	# data time stamp
State state             	# standard PackML state
int16 sub_state         	# non-standard state within standard PackML state (0 if no sub-state)
Mode mode               	# standard PackML mode
int16 error             	# TODO: define standard errors (probably in a separate msg)
int16 sub_error         	# non-standard/user defined error code

