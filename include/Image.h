
#ifndef __WW_IMAGE_HEADER__
#define __WW_IMAGE_HEADER__



namespace ww {



class Image {

public:

	enum DataType { UByte, Byte, UShort, Short, Int, Float32, Float64 };

	virtual void* data() { return 0; }
	virtual int width() { return 0; }
	virtual int height() { return 0; }
	
	virtual void gray(Image*& out) { }
	virtual void sobel_x(Image*& out) { }
	virtual void sobel_y(Image*& out) { }
	virtual void subtract(Image* b, Image*& out) { }
	virtual void copy_to(Image*& out) { }
	virtual void convert_to(Image*& out, DataType type) { }
	
	virtual void set(double v) { }
	virtual void save(const char* path) { }
	virtual void resize(Image*& out) { }
	virtual double average2() { return 0; }
	virtual double abs_mean() { return 0; }
	virtual DataType type() { return UByte; }
	virtual int channels() { return 0; }
	virtual bool empty() { return true; }
	//virtual float sample(const float& a, const float& b) { return 0; }
	

};


}

#endif
