.PHONY: compile upload
compile:
	pio run

upload:
	pio run -t upload
