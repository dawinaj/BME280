
idf_component_register(SRCS "main.cpp"
	INCLUDE_DIRS "." "include/"
	REQUIRES driver BME280_SensorAPI)

component_compile_options(-std=gnu++2a)
