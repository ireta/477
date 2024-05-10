rasterizer:
	g++ *.cpp -o rasterizer -std=c++11 -O3
clean:
	rm -f rasterizer
go:
	rm -f rasterizer
	clear
	g++ *.cpp -o rasterizer -std=c++11 -O3
	rm -fr *.ppm
	rm -fr *.png
	./rasterizer ../inputs_outputs/inputs_outputs/different_projection_type/horse_and_mug/horse_and_mug_perspective.xml
