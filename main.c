/*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>
#ifdef __linux__
#include <linux/serial.h>
#endif
#include <sys/ioctl.h>
#include <SDL/SDL.h>
#include <jpeglib.h>
#include <setjmp.h>
#define CLIP(X) ( (X) > 255 ? 255 : (X) < 0 ? 0 : X)

// RGB -> YUV
#define RGB2Y(R, G, B) CLIP(( (  66 * (R) + 129 * (G) +  25 * (B) + 128) >> 8) +  16)
#define RGB2U(R, G, B) CLIP(( ( -38 * (R) -  74 * (G) + 112 * (B) + 128) >> 8) + 128)
#define RGB2V(R, G, B) CLIP(( ( 112 * (R) -  94 * (G) -  18 * (B) + 128) >> 8) + 128)

// YUV -> RGB
#define C(Y) ( (Y) - 16  )
#define D(U) ( (U) - 128 )
#define E(V) ( (V) - 128 )

#define YUV2R(Y, U, V) CLIP(( 298 * C(Y)              + 409 * E(V) + 128) >> 8)
#define YUV2G(Y, U, V) CLIP(( 298 * C(Y) - 100 * D(U) - 208 * E(V) + 128) >> 8)
#define YUV2B(Y, U, V) CLIP(( 298 * C(Y) + 516 * D(U)              + 128) >> 8)

// RGB -> YCbCr
#define CRGB2Y(R, G, B) CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16)
#define CRGB2Cb(R, G, B) CLIP((36962 * (B - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)
#define CRGB2Cr(R, G, B) CLIP((46727 * (R - CLIP((19595 * R + 38470 * G + 7471 * B ) >> 16) ) >> 16) + 128)

// YCbCr -> RGB
#define CYCbCr2R(Y, Cb, Cr) CLIP( Y + ( 91881 * Cr >> 16 ) - 179 )
#define CYCbCr2G(Y, Cb, Cr) CLIP( Y - (( 22544 * Cb + 46793 * Cr ) >> 16) + 135)
#define CYCbCr2B(Y, Cb, Cr) CLIP( Y + (116129 * Cb >> 16 ) - 226 )
#define FRAME_VALUES 10

// An array to store frame times:
static uint32_t frametimes[FRAME_VALUES];

// Last calculated SDL_GetTicks
static uint32_t frametimelast;

// total frames rendered
static uint32_t framecount;

// the value you want
static float framespersecond;

// This function gets called once on startup.

static void fpsinit(){
	// Set all frame times to 0ms.
	memset(frametimes, 0, sizeof(frametimes));
	framecount = 0;
	framespersecond = 0;
	frametimelast = SDL_GetTicks();
}


static void fpsthink(){
	uint32_t frametimesindex;
	uint32_t getticks;
	uint32_t count;
	uint32_t i;

	// frametimesindex is the position in the array. It ranges from 0 to FRAME_VALUES.
	// This value rotates back to 0 after it hits FRAME_VALUES.
	frametimesindex = framecount % FRAME_VALUES;

	// store the current time
	getticks = SDL_GetTicks();

	// save the frame time value
	frametimes[frametimesindex] = getticks - frametimelast;

	// save the last frame time for the next fpsthink
	frametimelast = getticks;

	// increment the frame count
	framecount++;

	// Work out the current framerate

	// The code below could be moved into another function if you don't need the value every frame.

	// I've included a test to see if the whole array has been written to or not. This will stop
	// strange values on the first few (FRAME_VALUES) frames.
	if (framecount < FRAME_VALUES) {
		count = framecount;
	} else {
		count = FRAME_VALUES;
	}

	// add up all the values and divide to get the average frame time.
	framespersecond = 0;
	for (i = 0; i < count; i++) {
		framespersecond += frametimes[i];
	}

	framespersecond /= count;

	// now to make it an actual frames per second value...
	framespersecond = 1000.f / framespersecond;

}
static int readC(int fd,void *buffer,int nbyte){
	static int oldN;
	if(oldN!=nbyte){
		struct termios options;
		tcgetattr(fd, &options);
		options.c_cc[VMIN] = nbyte;/* block untill n bytes are received */
		tcsetattr(fd, TCSANOW, &options);
	}
	int ret=read(fd,buffer,nbyte);
	if(ret!=nbyte)
		perror("Read error");
	return ret;
}
static void waitImg(int fd){
	static const char Sig[]="RDY";
	uint8_t tempC;
	unsigned junkC=0;
	unsigned x;
	//puts("Waiting for command");
	for (x=0;x<3;++x){
		do{
			SDL_Event event;
			while( SDL_PollEvent( &event )){
				if( event.type == SDL_QUIT){
					//Quit the program
					close(fd);
					exit(1);
				}
			}
			readC(fd,&tempC,1);
			if(tempC != Sig[x]){
				++junkC;
			}
		}while (tempC != Sig[x]);
	}
	if(junkC)
		printf("%d junk bytes skipped\n",junkC);
}
enum COLORSPACE{RGB565,YUV422,RAW,JPEG};
struct baudDef{
	const char*str;
	speed_t baudRate;
};
static const struct baudDef baudTab[]={
	{"50",B50},
	{"75",B75},
	{"110",B110},
	{"134",B134},
	{"150",B150},
	{"200",B200},
	{"300",B300},
	{"600",B600},
	{"1200",B1200},
	{"1800",B1800},
	{"2400",B2400},
	{"4800",B4800},
	{"9600",B9600},
	{"19200",B19200},
	{"38400",B38400},
	{"57600",B57600},
	{"115200",B115200},
	{"230400",B230400},
	{"460800",B460800},
	{"500000",B500000},
	{"576000",B576000},
	{"921600",B921600},
	{"1000000",B1000000},
	{"1152000",B1152000},
	{"1500000",B1500000},
	{"2000000",B2000000},
	{"2500000",B2500000},
	{"3000000",B3000000},
	{"3500000",B3500000},
	{"4000000",B4000000}
};
static void*getJpeg(int fd,size_t*size){
	uint8_t*ptr=0;
	size_t sz=0;
	int look=0;
	for(;;){
		uint8_t b;
		readC(fd,&b,1);
		ptr=realloc(ptr,++sz);
		ptr[sz-1]=b;
		if(look){
			if(b==0xD9)
				break;
			else
				look=0;
		}
		if(b==0xFF)
			look=1;
	}
	if(size)
		*size=sz;
	printf("Size: %u\n",sz);
	return ptr;
}
struct my_error_mgr {
	struct jpeg_error_mgr pub;    /* "public" fields */

	jmp_buf setjmp_buffer;        /* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

/*
 * Here's the routine that will replace the standard error_exit method:
 */

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
	/* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
	my_error_ptr myerr = (my_error_ptr) cinfo->err;

	/* Always display the message. */
	/* We could postpone this until after returning, if we chose. */
	(*cinfo->err->output_message) (cinfo);

	/* Return control to the setjmp point */
	longjmp(myerr->setjmp_buffer, 1);
}
int main(int argc,char**argv){
	if(argc!=7){
		printf("Usage:\n%s width height colorspace baudrate deviceLocation protocolVersion\nValid colorspace options rgb565,yuv422,raw\nValid protocol options 0,1\n",argv[0]);
		return 1;
	}
	int width=atoi(argv[1]);
	if(width<=0){
		puts("Invalid width");
		return 1;
	}
	int height=atoi(argv[2]);
	if(height<=0){
		puts("Invalid height");
		return 1;
	}
	enum COLORSPACE colspace;
	if(strcmp(argv[3],"rgb565")==0)
		colspace=RGB565;
	else if(strcmp(argv[3],"yuv422")==0)
		colspace=YUV422;
	else if(strcmp(argv[3],"raw")==0)
		colspace=RAW;
	else if(strcmp(argv[3],"jpeg")==0)
		colspace=JPEG;
	else{
		puts("Invalid colorspace");
		return 1;
	}
	speed_t baudRate=0;
	{
		unsigned i;
		for(i=0;i<sizeof(baudTab)/sizeof(baudTab[0]);++i){
			if(strcmp(argv[4],baudTab[i].str)==0)
				baudRate=baudTab[i].baudRate;
		}
		if(!baudRate){
			puts("Invalid baud rate. Try adding it to the baud rate table to fix this message.");
			return 1;
		}
	}
	int protocolVersion=atoi(argv[6]);
	printf("%d %d %d %d %d\n",width,height,colspace,baudRate,protocolVersion);
	SDL_Surface* screen = NULL;
	SDL_Surface*image;
	if(colspace==RGB565)
		image = SDL_CreateRGBSurface(SDL_SWSURFACE,width,height,16,be16toh(0xF800),be16toh(0x07e0),be16toh(0x1F),0);
	else
		image = SDL_CreateRGBSurface(SDL_SWSURFACE,width,height,24,0x0000ff,0x00ff00,0xff0000,0);
	//Start SDL
	SDL_Init(SDL_INIT_EVERYTHING);
	//Set up screen
	screen = SDL_SetVideoMode(width,height, 32, SDL_SWSURFACE );
	SDL_Event event;
	struct termios options;
	int fd=open(argv[5],O_RDWR | O_NOCTTY);
	if(fd<0){
		perror("Cannot open");
		return 1;
	}
	tcgetattr(fd, &options);
	cfsetispeed(&options, baudRate);
	cfsetospeed(&options, baudRate);
	options.c_cflag = baudRate | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VMIN] = 1;      /* block until n bytes are received */
	options.c_cc[VTIME] = 0;     /* block until a timer expires (n * 100 mSec.) */
	tcsetattr(fd, TCSANOW, &options);
	tcflush(fd,TCIOFLUSH);
	sleep(1);
#ifdef __linux__
	{struct serial_struct serinfo;
	ioctl (fd, TIOCGSERIAL, &serinfo);
	serinfo.flags |= ASYNC_LOW_LATENCY;
	ioctl (fd, TIOCSSERIAL, &serinfo);}
#endif
	uint8_t tmpBuf[4];
	int exitLoop=1;
	fpsinit();
	if(colspace==JPEG){
		uint8_t*first=getJpeg(fd,NULL);//Skip partial frame
		free(first);
	}
	while(exitLoop){
		SDL_LockSurface(image);
		uint8_t*imgPtr=image->pixels;
		unsigned x,y=0,saveImg=0;
		if(!protocolVersion)
			waitImg(fd);
		size_t sz;
		uint8_t*jpg=0;
		if(colspace==JPEG){
			jpg=getJpeg(fd,&sz);
			printf("Jpeg grabbed size: %u\n",sz);

			/* This struct contains the JPEG decompression parameters and pointers to
			 * working space (which is allocated as needed by the JPEG library).
			 */
			struct jpeg_decompress_struct cinfo;
			struct my_error_mgr jerr;
			/* We use our private extension JPEG error handler.
			 * Note that this struct must live as long as the main JPEG parameter
			 * struct, to avoid dangling-pointer problems.
			 */
			/* More stuff */
			int row_stride;               /* physical row width in output buffer */


			/* Step 1: allocate and initialize JPEG decompression object */


			/* We set up the normal JPEG error routines, then override error_exit. */
			cinfo.err = jpeg_std_error(&jerr.pub);
			jerr.pub.error_exit = my_error_exit;
			/* Establish the setjmp return context for my_error_exit to use. */
			if (setjmp(jerr.setjmp_buffer)) {
				/* If we get here, the JPEG code has signaled an error.
				 * We need to clean up the JPEG object, close the input file, and return.
				 */
				jpeg_destroy_decompress(&cinfo);
				return 0;
			}
			/* Now we can initialize the JPEG compression object. */
			jpeg_create_compress(&cinfo);
			/* Now we can initialize the JPEG decompression object. */
			jpeg_create_decompress(&cinfo);

			/* Step 2: specify data source (eg, a file) */

			jpeg_mem_src(&cinfo,jpg,sz);

			/* Step 3: read file parameters with jpeg_read_header() */

			(void) jpeg_read_header(&cinfo, TRUE);
			/* We can ignore the return value from jpeg_read_header since
			 *   (a) suspension is not possible with the stdio data source, and
			 *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
			 * See libjpeg.txt for more info.
			 */

			/* Step 4: set parameters for decompression */

			/* In this example, we don't need to change any of the defaults set by
			 * jpeg_read_header(), so we do nothing here.
			 */

			/* Step 5: Start decompressor */

			(void) jpeg_start_decompress(&cinfo);
			/* We can ignore the return value since suspension is not possible
			 * with the stdio data source.
			 */

			/* We may need to do some setup of our own at this point before reading
			 * the data.  After jpeg_start_decompress() we have the correct scaled
			 * output image dimensions available, as well as the output colormap
			 * if we asked for color quantization.
			 * In this example, we need to make an output work buffer of the right size.
			 */
			/* JSAMPLEs per row in output buffer */
			row_stride = cinfo.output_width * cinfo.output_components;
			/* Make a one-row-high sample array that will go away when done with image */

			/* Step 6: while (scan lines remain to be read) */
			/*           jpeg_read_scanlines(...); */

			/* Here we use the library's state variable cinfo.output_scanline as the
			 * loop counter, so that we don't have to keep track ourselves.
			 */
			JSAMPROW row_pointer[1];
  			JSAMPARRAY buffer;            /* Output row buffer */
			buffer = (*cinfo.mem->alloc_sarray)
				((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

			/* Step 6: while (scan lines remain to be read) */
			/*           jpeg_read_scanlines(...); */

			/* Here we use the library's state variable cinfo.output_scanline as the
			 * loop counter, so that we don't have to keep track ourselves.
			 */
			while (cinfo.output_scanline < cinfo.output_height) {
				/* jpeg_read_scanlines expects an array of pointers to scanlines.
				 * Here the array is only one element long, but you could ask for
				 * more than one scanline at a time if that's more convenient.
				 */
				(void) jpeg_read_scanlines(&cinfo, buffer, 1);
				/* Assume put_scanline_someplace wants a pointer and sample count. */
				//put_scanline_someplace(buffer[0], row_stride);
				memcpy(image->pixels+(y*width*3),buffer[0],width*3);
				++y;
			}

			/* Step 7: Finish decompression */

			(void) jpeg_finish_decompress(&cinfo);
			/* We can ignore the return value since suspension is not possible
			 * with the stdio data source.
			 */

			/* Step 8: Release JPEG decompression object */

			/* This is an important step since it will release a good deal of memory. */
			jpeg_destroy_decompress(&cinfo);

			/* After finish_decompress, we can close the input file.
			 * Here we postpone it until after no more JPEG errors are possible,
			 * so as to simplify the setjmp error logic above.  (Actually, I don't
			 * think that jpeg_destroy can do an error exit, but why assume anything...)
			 */


		}else{
			for(y=0;y<height;++y){
				if(protocolVersion){
					waitImg(fd);
					uint16_t ln;
					readC(fd,&ln,2);
					ln%=height;
					//printf("%d\n",ln);
					y=ln;
					imgPtr=image->pixels+(image->pitch*ln);
				}
				for(x=0;x<width;++x){
					{
						int rdAmt,n;
						switch(colspace){
							case RGB565:
								rdAmt=2;
							break;
							case YUV422:
								if(x&1)
									continue;
								rdAmt=4;
							break;
							case RAW:
								rdAmt=1;
							break;
						}
						n = readC(fd,tmpBuf,rdAmt);
						if (n < rdAmt)
							printf("Read %d bytes instead of 4\n",n);
						if (n ==0){
							puts("0 bytes read");
							continue;
						}
					}
					switch(colspace){
						case RGB565:
							*imgPtr++=tmpBuf[0];
							*imgPtr++=tmpBuf[1];
						break;
						case YUV422:
							*imgPtr++=YUV2R(tmpBuf[1],tmpBuf[0],tmpBuf[2]);
							*imgPtr++=YUV2G(tmpBuf[1],tmpBuf[0],tmpBuf[2]);
							*imgPtr++=YUV2B(tmpBuf[1],tmpBuf[0],tmpBuf[2]);

							*imgPtr++=YUV2R(tmpBuf[3],tmpBuf[0],tmpBuf[2]);
							*imgPtr++=YUV2G(tmpBuf[3],tmpBuf[0],tmpBuf[2]);
							*imgPtr++=YUV2B(tmpBuf[3],tmpBuf[0],tmpBuf[2]);
						break;
						case RAW:
							//see what color this pixel should set to
							/*B G
							G R*/ 
							if(y&1){//odd
								if(x&1)//if odd
									imgPtr[0]=tmpBuf[0];//red
								else
									imgPtr[1]=tmpBuf[0];//green
							}else{//even
								if (x&1)//if odd pixel
									imgPtr[1]=tmpBuf[0];//green
								else
									imgPtr[2]=tmpBuf[0];//blue
							}
							imgPtr+=3;//next pixel
						break;
					}
				}
				if((colspace==RAW)&&(y&1)){
					//do bayer interpolation
					imgPtr=image->pixels+((y-1)*image->pitch);
					unsigned wx;
					for(wx=0;wx<width;wx+=2){
						//this will do a 2x2 pixel rectangle
						/*B G
						  G R*/ 
						imgPtr[wx*3]=imgPtr[((width+wx)*3)+3];//red
						imgPtr[1+wx*3]=imgPtr[4+wx*3];//green

						imgPtr[3+wx*3]=imgPtr[((width+wx)*3)+3];//red
						imgPtr[5+wx*3]=imgPtr[wx*3+2];//blue

						imgPtr[((width+wx)*3)]=imgPtr[((width+wx)*3)+3];//red
						imgPtr[((width+wx)*3)+2]=imgPtr[wx*3+2];//blue

						imgPtr[((width+wx)*3)+5]=imgPtr[wx*3+2];//blue
						imgPtr[((width+wx)*3)+4]=imgPtr[((width+wx)*3)+1];//green
					}
					imgPtr=image->pixels+(y*image->pitch);
				}
				if(!(y&15)){
					SDL_UnlockSurface(image);
					SDL_BlitSurface(image, NULL, screen, NULL );
					SDL_LockSurface(image);
					SDL_Flip(screen);
				}
				while(SDL_PollEvent(&event)){
					if(event.type==SDL_QUIT){
						//Quit the program
						exitLoop=0;
						goto quit;
					}
					if(event.type == SDL_KEYDOWN){
						if(event.key.keysym.sym == SDLK_s){
							puts("Saving Image");
							saveImg=1;
						}
					}
				}
			}
		}
		while(SDL_PollEvent(&event)){
			if(event.type==SDL_QUIT){
				//Quit the program
				exitLoop=0;
				goto quit;
			}
			if(event.type == SDL_KEYDOWN){
				if(event.key.keysym.sym == SDLK_s){
					puts("Saving Image");
					saveImg=1;
				}
			}
		}
		if(saveImg){
			saveImg=0;
			if(colspace==JPEG&&jpg){
				FILE*fp=fopen("OUT.jpg","wb");
				fwrite(jpg,sz,1,fp);
				fclose(fp);
			}else
			SDL_SaveBMP(image,"OUT.bmp");
		}
		free(jpg);
quit:
		SDL_UnlockSurface(image);
		SDL_BlitSurface(image, NULL, screen, NULL );
		SDL_Flip(screen);
		fpsthink();
		printf("Frames Per Second %f\n", framespersecond);
	}
	puts("Serial Port Closed");
	close(fd);
	SDL_FreeSurface(image);
	//Quit SDL
	SDL_Quit();
	putchar('\n');
	return 0;
}
