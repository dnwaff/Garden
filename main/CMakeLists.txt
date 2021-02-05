# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c" "app_wifi.c" "pwm.c" "plot.c" "event.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
