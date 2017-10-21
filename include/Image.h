
#ifndef __WW_IMAGE_HEADER__
#define __WW_IMAGE_HEADER__



namespace ww {



class Image {

public:

	enum DataType{ Float32 };

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
	

};


}

#endif
