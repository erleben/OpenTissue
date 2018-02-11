#ifndef OPENTISSUE_GPU_IMAGE_IO_IMAGE_IL_WRAP_H
#define OPENTISSUE_GPU_IMAGE_IO_IMAGE_IL_WRAP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

// NOTE: below code is pasted from il_wrap.h
//       Rememeber to update when a new version of devil is released...
//       <current: devil-1.6.7>

#include <IL/ilut.h>  // Probably only have to #include this one

namespace OpenTissue
{
  namespace image
  {
    namespace detail
    {


      void check_IL_errors()
      {
        ILenum Error;
        while ((Error = ilGetError()) != IL_NO_ERROR)
        {
          std::cerr << "check_IL_errors(): "
            << Error 
            << " : " 
            << iluErrorString(Error)
            << std::endl;
        }
      }


      std::string get_IL_string(ILenum code)
      {
        switch(code)
        {
          //case IL_COLOUR_INDEX  : return "IL_COLOUR_INDEX";
        case IL_COLOR_INDEX : return "IL_COLOR_INDEX";
        case IL_RGB : return "IL_RGB";
        case IL_RGBA: return "IL_RGBA";
        case IL_BGR : return "IL_BGR";
        case IL_BGRA: return "IL_BGRA";
        case IL_LUMINANCE : return "IL_LUMINANCE";
        case IL_LUMINANCE_ALPHA : return "IL_LUMINANCE_ALPHA";
        case IL_BYTE: return "IL_BYTE";
        case IL_UNSIGNED_BYTE: return "IL_UNSIGNED_BYTE";
        case IL_SHORT: return "IL_SHORT";
        case IL_UNSIGNED_SHORT: return "IL_UNSIGNED_SHORT";
        case IL_INT: return "IL_INT";
        case IL_UNSIGNED_INT: return "IL_UNSIGNED_INT";
        case IL_FLOAT: return "IL_FLOAT";
        case IL_DOUBLE: return "IL_DOUBLE";

        case IL_PAL_NONE: return "IL_PAL_NONE";
        case IL_PAL_RGB24: return "IL_PAL_RGB24";
        case IL_PAL_RGB32: return "IL_PAL_RGB32";
        case IL_PAL_RGBA32: return "IL_PAL_RGBA32";
        case IL_PAL_BGR24: return "IL_PAL_BGR24";
        case IL_PAL_BGR32: return "IL_PAL_BGR32";
        case IL_PAL_BGRA32: return "IL_PAL_BGRA32";
        case IL_TYPE_UNKNOWN: return "IL_TYPE_UNKNOWN";
        case IL_BMP: return "IL_BMP";
        case IL_CUT: return "IL_CUT";
        case IL_DOOM: return "IL_DOOM";
        case IL_DOOM_FLAT: return "IL_DOOM_FLAT";
        case IL_ICO: return "IL_ICO";
        case IL_JPG: return "IL_JPG";
          //    case IL_JFIF: return "IL_JFIF";
        //case IL_LBM: return "IL_LBM"; 2010-07-15 perb: Uncommented as this is renamed to IL_ILBM since version 1.7.5
        case IL_PCD: return "IL_PCD";
        case IL_PCX: return "IL_PCX";
        case IL_PIC: return "IL_PIC";
        case IL_PNG: return "IL_PNG";
        case IL_PNM: return "IL_PNM";
        case IL_SGI: return "IL_SGI";
        case IL_TGA: return "IL_TGA";
        case IL_TIF: return "IL_TIF";
        case IL_CHEAD: return "IL_CHEAD";
        case IL_RAW: return "IL_RAW";
        case IL_MDL: return "IL_MDL";
        case IL_WAL: return "IL_WAL";
        case IL_LIF: return "IL_LIF";
        case IL_MNG: return "IL_MNG";
          //    case IL_JNG: return "IL_JNG";
        case IL_GIF: return "IL_GIF";
        case IL_DDS: return "IL_DDS";
        case IL_DCX: return "IL_DCX";
        case IL_PSD: return "IL_PSD";
        case IL_EXIF: return "IL_EXIF";
        case IL_PSP: return "IL_PSP";
        case IL_PIX: return "IL_PIX";
        case IL_PXR: return "IL_PXR";
        case IL_XPM: return "IL_XPM";
        case IL_JASC_PAL: return "IL_JASC_PAL";
        case IL_ORIGIN_SET: return "IL_ORIGIN_SET";
        case IL_ORIGIN_LOWER_LEFT: return "IL_ORIGIN_LOWER_LEFT";
        case IL_ORIGIN_UPPER_LEFT: return "IL_ORIGIN_UPPER_LEFT";
        case IL_ORIGIN_MODE: return "IL_ORIGIN_MODE";
        case IL_FORMAT_SET: return "IL_FORMAT_SET";
        case IL_FORMAT_MODE: return "IL_FORMAT_MODE";
        case IL_TYPE_SET: return "IL_TYPE_SET";
        case IL_TYPE_MODE: return "IL_TYPE_MODE";
        };
        return "unregonized";
      }

      class ilImage
      {
      public:
        ilImage()
        {
          this->Id = 0;
          //this->iStartUp();
          this->iGenBind();
          return;
        }

        ilImage(char *FileName)
        {
          this->Id = 0;
          //this->iStartUp();
          this->iGenBind();
          ilLoadImage(FileName);
          return;
        }

        ilImage(const ilImage &Image)
        {
          this->Id = 0;
          //this->iStartUp();
          this->iGenBind();
          *this = Image;
          return;
        }

        ~ilImage()
        {
          if (this->Id)
            ilDeleteImages(1, &this->Id);
          this->Id = 0;
          return;
        }

        ILboolean Load(char *FileName)
        {
          this->iGenBind();
          return ilLoadImage(FileName);
        }

        ILboolean Load(char *FileName, ILenum type)
        {
          this->iGenBind();
          return ilLoad(type, FileName);
        }

        ILboolean Save(char *FileName)
        {
          this->iGenBind();
          return ilSaveImage(FileName);
        }

        ILboolean Save(char *FileName, ILenum type)
        {
          this->iGenBind();
          return ilSave(type, FileName);
        }

        ////////////////////////////////////////////////////////////////
        // ImageLib functions
        ////////////////////////////////////////////////////////////////

        ILboolean ActiveImage(ILuint Number)
        {
          if (this->Id) {
            this->Bind();
            return ilActiveImage(Number);
          }
          return IL_FALSE;
        }

        ILboolean ActiveLayer(ILuint Number)
        {
          if (this->Id) {
            this->Bind();
            return ilActiveLayer(Number);
          }
          return IL_FALSE;
        }

        ILboolean ActiveMipmap(ILuint Number)
        {
          if (this->Id) {
            this->Bind();
            return ilActiveMipmap(Number);
          }
          return IL_FALSE;
        }

        ILboolean Clear()
        {
          if (this->Id) {
            this->Bind();
            return ilClearImage();
          }
          return IL_FALSE;
        }

        void ClearColour(ILubyte Red, ILubyte Green, ILubyte Blue, ILubyte Alpha)
        {
          ilClearColour(Red, Green, Blue, Alpha);
          return;
        }

        ILboolean Convert(ILenum NewFormat)
        {
          if (this->Id) {
            this->Bind();
            return ilConvertImage(NewFormat, IL_UNSIGNED_BYTE);
          }
          return IL_FALSE;
        }

        ILboolean Copy(ILuint Src)
        {
          if (this->Id) {
            this->Bind();
            return ilCopyImage(Src);
          }
          return IL_FALSE;
        }

        ILboolean Default()
        {
          if (this->Id) {
            this->Bind();
            return ilDefaultImage();
          }
          return IL_FALSE;
        }

        ILboolean Flip()
        {
          if (this->Id) {
            this->Bind();
            return iluFlipImage();
          }
          return IL_FALSE;
        }

        ILboolean SwapColours()
        {
          if (this->Id) {
            this->Bind();
            return iluSwapColours();
          }
          return IL_FALSE;
        }

        ILboolean Resize(ILuint width, ILuint height, ILuint depth)
        {
          if (this->Id) {
            this->Bind();
            return iluScale(width, height, depth);
          }
          return IL_FALSE;
        }

        ILboolean TexImage(ILuint width, ILuint height, ILuint depth, ILubyte bpp, ILenum format, ILenum type, void *data)
        {
          if (this->Id) {
            this->Bind();
            return ilTexImage(width, height, depth, bpp, format, type, data);
          }
          return IL_FALSE;
        }

        ////////////////////////////////////////////////////////////////
        // Image handling
        ////////////////////////////////////////////////////////////////

        void Bind() const
        {
          if (this->Id)
            ilBindImage(this->Id);
          return;
        }

        // Note:  Behaviour may be changed!
        void Bind(ILuint Image)
        {
          if (this->Id == Image)
            return;
          this->Delete();  // Should we delete it?
          this->Id = Image;
          ilBindImage(this->Id);
          return;
        }

        void Delete()
        {
          if (this->Id == 0)
            return;
          ilDeleteImages(1, &this->Id);
          this->Id = 0;
          return;
        }

        ////////////////////////////////////////////////////////////////
        // Image characteristics
        ////////////////////////////////////////////////////////////////

        ILuint Width()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_IMAGE_WIDTH);
          }
          return 0;
        }

        ILuint Height()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_IMAGE_HEIGHT);
          }
          return 0;
        }

        ILuint Depth()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_IMAGE_DEPTH);
          }
          return 0;
        }

        ILubyte Bpp()
        {
          if (this->Id) {
            this->Bind();
            return static_cast<ILubyte>( ilGetInteger(IL_IMAGE_BYTES_PER_PIXEL) );
          }
          return 0;
        }

        ILubyte Bitpp()
        {
          if (this->Id) {
            this->Bind();
            return static_cast<ILubyte>( ilGetInteger(IL_IMAGE_BITS_PER_PIXEL) );
          }
          return 0;
        }

        ILenum Format()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_IMAGE_FORMAT);
          }
          return 0;
        }

        ILenum PaletteType()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_PALETTE_TYPE);
          }
          return 0;
        }

        ILenum Type()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_IMAGE_TYPE);
          }
          return 0;
        }

        ILenum NumImages()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_NUM_IMAGES);
          }
          return 0;
        }

        ILenum NumMipmaps()
        {
          if (this->Id) {
            this->Bind();
            return ilGetInteger(IL_NUM_MIPMAPS);
          }
          return 0;
        }

        ILuint GetId() const
        {
          return this->Id;
        }

        ILenum GetOrigin()
        {
          ILinfo Info;

          if (this->Id) {
            this->Bind();
            iluGetImageInfo(&Info);
            return Info.Origin;
          }
          return 0;
        }

        ILubyte* GetData()
        {
          if (this->Id) {
            this->Bind();
            return ilGetData();
          }
          return 0;
        }

        ILubyte* GetPalette()
        {
          if (this->Id) {
            this->Bind();
            return ilGetPalette();
          }
          return 0;
        }

        void iGenBind()
        {
          if (this->Id == 0) {
            ilGenImages(1, &this->Id);
          }
          ilBindImage(this->Id);
          return;
        }

        ////////////////////////////////////////////////////////////////
        // Operators
        ////////////////////////////////////////////////////////////////

        ilImage& operator = (ILuint Image)
        {
          if (this->Id == 0)
            this->Id = Image;
          else {
            this->Bind();
            ilCopyImage(Image);
          }

          return *this;
        }

        ilImage& operator = (const ilImage &Image)
        {
          if (Id == 0)
            Id = Image.GetId();
          else {
            Bind();
            ilCopyImage(Image.GetId());
          }

          return *this;
        }

      protected:
        ILuint    Id;

      private:
        ////////////////////////////////////////////////////////////////
        // Private members
        ////////////////////////////////////////////////////////////////

        void iStartUp()
        {
          ilInit();
          iluInit();
          ilutInit();
          return;
        }

      };

      class ilFilters
      {
      public:
        ////////////////////////////////////////////////////////////////
        // ILFILTERS
        ////////////////////////////////////////////////////////////////

        static ILboolean Alienify(ilImage &Image)
        {
          Image.Bind();
          return iluAlienify();
        }

        static ILboolean BlurAvg(ilImage &Image, ILuint Iter)
        {
          Image.Bind();
          return iluBlurAvg(Iter);
        }

        static ILboolean BlurGaussian(ilImage &Image, ILuint Iter)
        {
          Image.Bind();
          return iluBlurGaussian(Iter);
        }

        static ILboolean Contrast(ilImage &Image, ILfloat Contrast)
        {
          Image.Bind();
          return iluContrast(Contrast);
        }

        static ILboolean EdgeDetectE(ilImage &Image)
        {
          Image.Bind();
          return iluEdgeDetectP();
        }

        static ILboolean EdgeDetectP(ilImage &Image)
        {
          Image.Bind();
          return iluEdgeDetectP();
        }

        static ILboolean EdgeDetectS(ilImage &Image)
        {
          Image.Bind();
          return iluEdgeDetectS();
        }

        static ILboolean Emboss(ilImage &Image)
        {
          Image.Bind();
          return iluEmboss();
        }

        static ILboolean Gamma(ilImage &Image, ILfloat Gamma)
        {
          Image.Bind();
          return iluGammaCorrect(Gamma);
        }

        static ILboolean Negative(ilImage &Image)
        {
          Image.Bind();
          return iluNegative();
        }

        static ILboolean Noisify(ilImage &Image, ILubyte Factor)
        {
          Image.Bind();
          return iluNoisify(Factor);
        }

        static ILboolean Pixelize(ilImage &Image, ILuint PixSize)
        {
          Image.Bind();
          return iluPixelize(PixSize);
        }

        static ILboolean Saturate(ilImage &Image, ILfloat Saturation)
        {
          Image.Bind();
          return iluSaturate1f(Saturation);
        }

        static ILboolean Saturate(ilImage &Image, ILfloat r, ILfloat g, ILfloat b, ILfloat Saturation)
        {
          Image.Bind();
          return iluSaturate4f(r, g, b, Saturation);
        }

        static ILboolean ScaleColours(ilImage &Image, ILfloat r, ILfloat g, ILfloat b)
        {
          Image.Bind();
          return iluScaleColours(r, g, b);
        }

        static ILboolean Sharpen(ilImage &Image, ILfloat Factor, ILuint Iter)
        {
          Image.Bind();
          return iluSharpen(Factor, Iter);
        }

      };


#ifdef ILUT_USE_OPENGL
      class ilOgl
      {
      public:
        static void Init()
        {
          ilutRenderer(ILUT_OPENGL);
          return;
        }

        static GLuint BindTex(ilImage &Image)
        {
          Image.Bind();
          return ilutGLBindTexImage();
        }

        static ILboolean Upload(ilImage &Image, ILuint Level)
        {
          Image.Bind();
          return ilutGLTexImage(Level);
        }

        static GLuint Mipmap(ilImage &Image)
        {
          Image.Bind();
          return ilutGLBuildMipmaps();
        }

        static ILboolean Screen()
        {
          return ilutGLScreen();
        }

        static ILboolean Screenie()
        {
          return ilutGLScreenie();
        }
      };
#endif//ILUT_USE_OPENGL


#ifdef ILUT_USE_ALLEGRO
      class ilAlleg
      {
      public:
        static void Init()
        {
          ilutRenderer(IL_ALLEGRO);
          return;
        }

        static BITMAP * Convert(ilImage &Image, PALETTE Pal)
        {
          Image.Bind();
          return ilutConvertToAlleg(Pal);
        }
      };
#endif//ILUT_USE_ALLEGRO


#ifdef ILUT_USE_WIN32
      class ilWin32
      {
      public:
        static void Init()
        {
          ilutRenderer(ILUT_WIN32);
          return;
        }

        static HBITMAP Convert(ilImage &Image)
        {
          Image.Bind();
          return ilutConvertToHBitmap(GetDC(NULL));
        }

        static ILboolean GetClipboard(ilImage &Image)
        {
          Image.Bind();
          return ilutGetWinClipboard();
        }

        static void GetInfo(ilImage &Image, BITMAPINFO *Info)
        {
          Image.Bind();
          ilutGetBmpInfo(Info);
          return;
        }

        static ILubyte* GetPadData(ilImage &Image)
        {
          Image.Bind();
          return ilutGetPaddedData();
        }

        static HPALETTE GetPal(ilImage &Image)
        {
          Image.Bind();
          return ilutGetHPal();
        }

        static ILboolean GetResource(ilImage &Image, HINSTANCE hInst, ILint ID, char *ResourceType)
        {
          Image.Bind();
          return ilutLoadResource(hInst, ID, ResourceType, IL_TYPE_UNKNOWN);
        }

        static ILboolean GetResource(ilImage &Image, HINSTANCE hInst, ILint ID, char *ResourceType, ILenum Type)
        {
          Image.Bind();
          return ilutLoadResource(hInst, ID, ResourceType, Type);
        }

        static ILboolean SetClipboard(ilImage &Image)
        {
          Image.Bind();
          return ilutSetWinClipboard();
        }
      };
#endif//ILUT_USE_WIN32


      class ilValidate
      {
      public:
        static ILboolean Valid(ILenum Type, char *FileName)
        {
          return ilIsValid(Type, FileName);
        }

        static ILboolean Valid(ILenum Type, FILE *File)
        {
          return ilIsValidF(Type, File);
        }

        static ILboolean Valid(ILenum Type, void *Lump, ILuint Size)
        {
          return ilIsValidL(Type, Lump, Size);
        }

      protected:

      private:

      };


      class ilState
      {
      public:
        static ILboolean Disable(ILenum State)
        {
          return ilDisable(State);
        }

        static ILboolean Enable(ILenum State)
        {
          return ilEnable(State);
        }

        static void Get(ILenum Mode, ILboolean &Param)
        {
          ilGetBooleanv(Mode, &Param);
          return;
        }

        static void Get(ILenum Mode, ILint &Param)
        {
          ilGetIntegerv(Mode, &Param);
          return;
        }

        static ILboolean GetBool(ILenum Mode)
        {
          return ilGetBoolean(Mode);
        }

        static ILint GetInt(ILenum Mode)
        {
          return ilGetInteger(Mode);
        }

        static const char *GetString(ILenum StringName)
        {
          return ilGetString(StringName);
        }

        static ILboolean IsDisabled(ILenum Mode)
        {
          return ilIsDisabled(Mode);
        }

        static ILboolean IsEnabled(ILenum Mode)
        {
          return ilIsEnabled(Mode);
        }

        static ILboolean Origin(ILenum Mode)
        {
          return ilOriginFunc(Mode);
        }

        static void Pop()
        {
          ilPopAttrib();
          return;
        }

        static void Push(ILuint Bits = IL_ALL_ATTRIB_BITS)
        {
          ilPushAttrib(Bits);
          return;
        }


      protected:

      private:

      };


      class ilError
      {
      public:
        static void Check(void (*Callback)(const char*))
        {
          static ILenum Error;

          while ((Error = ilGetError()) != IL_NO_ERROR) {
            Callback(iluErrorString(Error));
          }

          return;
        }

        static void Check(void (*Callback)(ILenum))
        {
          static ILenum Error;

          while ((Error = ilGetError()) != IL_NO_ERROR) {
            Callback(Error);
          }

          return;
        }

        static ILenum Get()
        {
          return ilGetError();
        }

        static const char *String()
        {
          return iluErrorString(ilGetError());
        }

        static const char *String(ILenum Error)
        {
          return iluErrorString(Error);
        }

      protected:

      private:

      };

    } // namespace detail
  } // namespace image
} // namespace OpenTissue

// OPENTISSUE_GPU_IMAGE_IO_IMAGE_IL_WRAP_H
#endif
