set(MainFolder "$ENV{MAINFOLDER}")

idf_component_register(SRCS "${MainFolder}/main.cpp"
                      INCLUDE_DIRS ".")
