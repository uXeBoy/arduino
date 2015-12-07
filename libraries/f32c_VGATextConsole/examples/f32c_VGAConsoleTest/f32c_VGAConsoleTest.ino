#include <f32c_VGATextConsole.h>
#include <f32c_PS2Keyboard.h>

// CAUTION: The sketch below is a "hack" for testing purposes.  You have been warned. :-)

void drawRLE();

#define FB_WIDTH  640
#define FB_HEIGHT 480

//uint8_t fb[FB_WIDTH * FB_HEIGHT] __attribute__ ((aligned (4)));
uint8_t *fb = (uint8_t) 0x81f00000;

inline void plot(int x, int y, int color)
{
  uint32_t off = (y * FB_WIDTH) + x;
  if (off < sizeof (fb))
    fb[off] = color;
}

PS2Keyboard keyboard;

void setup() {
  // put your setup code here, to run once:
  vga.Setup();
  vga.SetColor(0x1F);
  vga.Clear();
  vga.DisableTextCursor();
  vga.println("Welcome to f32c_VGAConsoleTest");

  vga.println("\nVGA_textmode Features Configured:\n");

  vga.print("Video Generation:                 ");
  vga.println(vga.IsDisplayConfigured() ? "Yes" : "No");

  vga.print("Screen size:                      ");
  vga.print(vga.GetDisplayWidth());
  vga.print("x");
  vga.println(vga.GetDisplayHeight());

  vga.print("BRAM memory:                      ");
  vga.print(vga.GetBRAMSize());
  vga.println(" KB");
  
  vga.print("BRAM read register interface:     ");
  vga.println(vga.IsBRAMRegisterReadConfigured() ? "Yes" : "No");

  vga.print("Text Generation:                  ");
  vga.println(vga.IsTextConfigured() ? "Yes" : "No");

  if (vga.IsTextConfigured())
  {
	vga.print("Text Color Attribute Byte:        ");
	vga.println(!vga.IsMonochromeConfigured() ? "Yes" : "No");

	vga.print("Text screen size:                 ");
	vga.print(vga.GetTextDisplayWidth());
	vga.print("x");
	vga.println(vga.GetTextDisplayHeight());

	vga.print("Text Address:                     0x");
	vga.PrintHex((uint32_t)vga.GetTextAddress());
	vga.println("");

	vga.print("Font:                             ");
	vga.print(vga.GetFontWidth());
	vga.print("x");
	vga.print(vga.GetFontHeight());
	vga.print(" ");
	vga.print(vga.GetFontCharacters());
	vga.print(" chars (");
	vga.print((vga.GetFontHeight() * vga.GetFontCharacters())/1024);
	vga.println(" KB)");

	vga.print("Font Address:                     0x");
	vga.PrintHex((uint32_t)vga.GetFontAddress());
	vga.println("");

	vga.print("Font cell height                  ");
	vga.println((uint32_t)VGAText_GetTextCellHeight());

	vga.print("Cursor Generation:                ");
	vga.println(vga.IsCursorConfigured() ? "Yes" : "No");

	vga.print("Cursor Blink Generation:          ");
	vga.println(vga.IsCursorBlinkConfigured() ? "Yes" : "No");
  }
  vga.print("16 Color Palette:                 ");
  vga.println(vga.IsPaletteConfigured() ? "Yes" : "No");

  vga.print("Bitmap Generation:                ");
  vga.println(vga.IsBitmapConfigured() ? "Yes" : "No");

  if (vga.IsBitmapConfigured())
  {
    Serial.print("Bitmap Address:                   0x");
    Serial.println((uint32_t)vga.GetBitmapAddress(), HEX);
    vga.print("Bitmap Address:                   0x");
    vga.PrintHex((uint32_t)vga.GetBitmapAddress());
    vga.println("");
  }

  
  bool kbd = keyboard.begin();
//  vga.println(kbd ? "Keyboard Test" : "No Keyboard");

  vga.SetColor(0x17);
  vga.print("Hi Davor!  ");
  vga.SetColor(0x19);
  vga.print("Xark");
  vga.SetColor(0x17);
  vga.println(" here.");
  vga.SetColor(0x1F);
  vga.print("This is VGA_textmode 640x480 HDMI running on the fabulous f32c!");
  vga.SetColor(0x14);
  vga.println(" :-)");
  vga.SetColor(0x1A);
  vga.println("Bitmap mode with FIFO achieved! :-)");
  vga.SetColor(0x1F);
  vga.println("2, 4 or 16-color with optional palette and 8-bit (RRRGGGBB) achieved! :-)");
  vga.SetColor(0x1e);
  vga.println("Text+color with FIFO achieved (with help from Davor)! :-)");

  vga.SetColor(0x00);
  vga.SetPos(7, 26);
  vga.print("                                                                  ");
  vga.SetPos(7, 27);
  vga.print("                                                                  ");
  vga.SetPos(7, 28);
  vga.print("                                                                  ");
  vga.SetPos(28, 29);
  vga.SetColor(0x1f);
  vga.print("256 direct-color RRRGGGBB");

  if (vga.IsBitmapConfigured())
  {
	  vga.SetBitmapAddress(fb);
	  vga.SetBitmapColor(0x108010);
	  vga.EnableBitmap();
	  drawRLE();
	  for (int x = 0; x < 256; x++)
	  {
		for (int y = 0; y < 8; y++)
		{
		  plot(64+(x*2), 424+y, (x&0xfc));
		  plot(65+(x*2), 424+y, (x&0xfc));

		  plot(64+(x*2), 432+y, (x&0xfc)|1);
		  plot(65+(x*2), 432+y, (x&0xfc)|1);
		
		  plot(64+(x*2), 440+y, (x&0xfc)|2);
		  plot(65+(x*2), 440+y, (x&0xfc)|2);

		  plot(64+(x*2), 448+y, (x&0xfc)|3);
		  plot(65+(x*2), 448+y, (x&0xfc)|3);
		}
    }
  }
}

uint32_t cnt;
void loop() {

//  vga.print("Woo-hoo foo-bar and a high-kittie-ditty-ho! ");
  // put your main code here, to run repeatedly:
  if (keyboard.available()) {
    
    // read the next key
    int c = keyboard.read();	
    
    // check for some of the special keys
    if (c == PS2_ENTER) {
      vga.println();
    } else if (c == PS2_TAB) {
      vga.print("[Tab]");
    } else if (c == PS2_ESC) {
      vga.print("[ESC]");
    } else if (c == PS2_PAGEDOWN) {
      vga.print("[PgDn]");
    } else if (c == PS2_PAGEUP) {
      vga.print("[PgUp]");
    } else if (c == PS2_LEFTARROW) {
      vga.print("[Left]");
    } else if (c == PS2_RIGHTARROW) {
      vga.print("[Right]");
    } else if (c == PS2_UPARROW) {
      vga.print("[Up]");
    } else if (c == PS2_DOWNARROW) {
      vga.print("[Down]");
    } else if (c == PS2_DELETE) {
      vga.print("[Del]");
    } else if (c == PS2_INSERT) {
      vga.print("[Insert]");
    } else if (c == PS2_HOME) {
      vga.print("[Home]");
    } else if (c == PS2_END) {
      vga.print("[End]");
//    } else if (c == PS2_BACKSPACE) {
//      vga.print("[Backspace]");
    } else if (c == PS2_F1) {
      vga.print("[F1]");
    } else if (c == PS2_F2) {
      vga.print("[F2]");
    } else if (c == PS2_F3) {
      vga.print("[F3]");
    } else if (c == PS2_F4) {
      vga.print("[F4]");
    } else if (c == PS2_F5) {
      vga.print("[F5]");
    } else if (c == PS2_F6) {
      vga.print("[F6]");
    } else if (c == PS2_F7) {
      vga.print("[F7]");
    } else if (c == PS2_F8) {
      vga.print("[F8]");
    } else if (c == PS2_F9) {
      vga.print("[F9]");
    } else if (c == PS2_F10) {
      vga.print("[F10]");
    } else if (c == PS2_F11) {
      vga.print("[F11]");
    } else if (c == PS2_F12) {
      vga.print("[F12]");
    } else if (c) {
      if (c < ' ')
	  {
		vga.print("[");
		vga.PrintHex((uint8_t)c);
		vga.print("]");
	  }
      // otherwise, just print all normal characters
      vga.print((char)c);
    }
  }
}


// Compressed RLE data
const uint8_t HaDLogo[] =
{
	0x00, 0x82, 0x81, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x80, 0xae, 0x13, 0x80, 0xfb, 0x13, 0x80, 0xaf, 0x02, 0x80, 0xac, 0x19, 0x80, 0xf3, 0x19, 
	0x80, 0xad, 0x02, 0x80, 0xad, 0x1b, 0x80, 0xed, 0x1b, 0x80, 0xae, 0x02, 0x80, 0xae, 0x1c, 0x80, 
	0xe9, 0x1c, 0x80, 0xaf, 0x02, 0x80, 0xaf, 0x1d, 0x80, 0xe5, 0x1d, 0x80, 0xb0, 0x02, 0x80, 0xb0, 
	0x1e, 0x80, 0xe1, 0x1e, 0x80, 0xb1, 0x02, 0x80, 0xb1, 0x1f, 0x80, 0xdd, 0x1f, 0x80, 0xb2, 0x02, 
	0x80, 0xb2, 0x1f, 0x80, 0xdb, 0x1f, 0x80, 0xb3, 0x02, 0x80, 0xb3, 0x20, 0x80, 0xd7, 0x20, 0x80, 
	0xb4, 0x02, 0x80, 0xb5, 0x1f, 0x80, 0xd5, 0x1f, 0x80, 0xb6, 0x02, 0x80, 0xb6, 0x1f, 0x80, 0xd3, 
	0x1f, 0x80, 0xb7, 0x02, 0x80, 0xb7, 0x20, 0x80, 0xcf, 0x20, 0x80, 0xb8, 0x02, 0x80, 0xb8, 0x20, 
	0x80, 0xcd, 0x20, 0x80, 0xb9, 0x02, 0x80, 0xb9, 0x20, 0x80, 0xcb, 0x20, 0x80, 0xba, 0x02, 0x80, 
	0xba, 0x20, 0x80, 0xc9, 0x20, 0x80, 0xbb, 0x02, 0x80, 0xbb, 0x1f, 0x80, 0xc9, 0x1f, 0x80, 0xbc, 
	0x02, 0x80, 0xbc, 0x1f, 0x80, 0xc7, 0x1f, 0x80, 0xbd, 0x02, 0x80, 0xbe, 0x1e, 0x80, 0xc5, 0x1e, 
	0x80, 0xbf, 0x02, 0x80, 0xbf, 0x1e, 0x80, 0xc3, 0x1e, 0x80, 0xc0, 0x02, 0x80, 0xc0, 0x1e, 0x80, 
	0xc1, 0x1e, 0x80, 0xc1, 0x02, 0x80, 0xc1, 0x1d, 0x80, 0xc1, 0x1d, 0x80, 0xc2, 0x02, 0x80, 0xc2, 
	0x1d, 0x80, 0xbf, 0x1d, 0x80, 0xc3, 0x02, 0x80, 0xc3, 0x1c, 0x80, 0xbf, 0x1c, 0x80, 0xc4, 0x02, 
	0x80, 0xc4, 0x1c, 0x80, 0xbd, 0x1c, 0x80, 0xc5, 0x02, 0x80, 0xc5, 0x1c, 0x80, 0xbb, 0x1c, 0x80, 
	0xc6, 0x02, 0x80, 0xc7, 0x1a, 0x80, 0xbb, 0x1a, 0x80, 0xc8, 0x02, 0x80, 0xc8, 0x1a, 0x80, 0xb9, 
	0x1a, 0x80, 0xc9, 0x02, 0x80, 0xc8, 0x1a, 0x80, 0xb9, 0x1a, 0x80, 0xc9, 0x02, 0x80, 0xc7, 0x1b, 
	0x80, 0xb9, 0x1b, 0x80, 0xc8, 0x02, 0x80, 0xc6, 0x1d, 0x80, 0xb7, 0x1d, 0x80, 0xc7, 0x02, 0x80, 
	0xc5, 0x1e, 0x80, 0xb7, 0x1e, 0x80, 0xc6, 0x02, 0x80, 0xc4, 0x1f, 0x80, 0xb7, 0x1f, 0x80, 0xc5, 
	0x02, 0x80, 0xc3, 0x21, 0x80, 0xb5, 0x21, 0x80, 0xc4, 0x02, 0x80, 0xc2, 0x22, 0x80, 0xb5, 0x22, 
	0x80, 0xc3, 0x02, 0x80, 0xc1, 0x23, 0x80, 0xb5, 0x23, 0x80, 0xc2, 0x02, 0x80, 0xc0, 0x25, 0x80, 
	0xb3, 0x25, 0x80, 0xc1, 0x02, 0x80, 0xbf, 0x26, 0x80, 0xb3, 0x26, 0x80, 0xc0, 0x02, 0x80, 0x8a, 
	0x01, 0x34, 0x26, 0x80, 0xb3, 0x26, 0x34, 0x01, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x02, 0x32, 0x27, 
	0x80, 0xb3, 0x27, 0x32, 0x02, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x03, 0x30, 0x28, 0x80, 0xb3, 0x28, 
	0x30, 0x03, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x05, 0x2d, 0x29, 0x80, 0xb3, 0x29, 0x2d, 0x05, 0x80, 
	0x8b, 0x02, 0x80, 0x89, 0x07, 0x2b, 0x2b, 0x80, 0xb1, 0x2b, 0x2b, 0x07, 0x80, 0x8a, 0x02, 0x80, 
	0x89, 0x08, 0x29, 0x2c, 0x80, 0xb1, 0x2c, 0x29, 0x08, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x09, 0x27, 
	0x2d, 0x80, 0xb1, 0x2d, 0x27, 0x09, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0a, 0x25, 0x2e, 0x80, 0xb1, 
	0x2e, 0x25, 0x0a, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0b, 0x23, 0x2f, 0x80, 0xb1, 0x2f, 0x23, 0x0b, 
	0x80, 0x8a, 0x02, 0x80, 0x89, 0x0c, 0x21, 0x30, 0x80, 0xb1, 0x30, 0x21, 0x0c, 0x80, 0x8a, 0x02, 
	0x80, 0x89, 0x0d, 0x20, 0x30, 0x80, 0xb1, 0x30, 0x1f, 0x0e, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0f, 
	0x1d, 0x31, 0x80, 0xb1, 0x31, 0x1d, 0x0f, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x10, 0x1b, 0x32, 0x80, 
	0xb1, 0x32, 0x1b, 0x10, 0x80, 0x8a, 0x02, 0x80, 0x8a, 0x10, 0x19, 0x32, 0x80, 0xb3, 0x32, 0x19, 
	0x10, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x11, 0x17, 0x33, 0x80, 0xb3, 0x33, 0x17, 0x11, 0x80, 0x8b, 
	0x02, 0x80, 0x8a, 0x12, 0x15, 0x34, 0x80, 0xb3, 0x34, 0x15, 0x12, 0x80, 0x8b, 0x02, 0x80, 0x8a, 
	0x13, 0x13, 0x36, 0x80, 0xb1, 0x36, 0x13, 0x13, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x14, 0x11, 0x39, 
	0x80, 0xad, 0x39, 0x11, 0x14, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x16, 0x0e, 0x3b, 0x80, 0xab, 0x3b, 
	0x0e, 0x16, 0x80, 0x8b, 0x02, 0x80, 0x8b, 0x16, 0x0c, 0x3d, 0x80, 0xa9, 0x3d, 0x0c, 0x16, 0x80, 
	0x8c, 0x02, 0x80, 0x8b, 0x17, 0x0b, 0x3e, 0x80, 0xa7, 0x3e, 0x0b, 0x17, 0x80, 0x8c, 0x02, 0x80, 
	0x8b, 0x18, 0x09, 0x40, 0x80, 0xa5, 0x40, 0x09, 0x18, 0x80, 0x8c, 0x02, 0x80, 0x8b, 0x19, 0x07, 
	0x42, 0x80, 0xa3, 0x42, 0x07, 0x19, 0x80, 0x8c, 0x02, 0x80, 0x8c, 0x19, 0x05, 0x44, 0x80, 0xa1, 
	0x44, 0x05, 0x19, 0x80, 0x8d, 0x02, 0x80, 0x8c, 0x1a, 0x03, 0x46, 0x80, 0x9f, 0x46, 0x03, 0x1a, 
	0x80, 0x8d, 0x02, 0x80, 0x8d, 0x1a, 0x01, 0x49, 0x80, 0x9b, 0x49, 0x01, 0x1a, 0x80, 0x8e, 0x02, 
	0x80, 0x8d, 0x65, 0x80, 0x99, 0x65, 0x80, 0x8e, 0x02, 0x80, 0x8d, 0x66, 0x80, 0x97, 0x66, 0x80, 
	0x8e, 0x02, 0x80, 0x8e, 0x66, 0x80, 0x95, 0x66, 0x80, 0x8f, 0x02, 0x80, 0x8e, 0x67, 0x80, 0x93, 
	0x67, 0x80, 0x8f, 0x02, 0x80, 0x8f, 0x67, 0x80, 0x91, 0x67, 0x80, 0x90, 0x02, 0x80, 0x8f, 0x68, 
	0x80, 0x8f, 0x68, 0x80, 0x90, 0x02, 0x80, 0x90, 0x68, 0x80, 0x8d, 0x68, 0x80, 0x91, 0x02, 0x80, 
	0x91, 0x69, 0x80, 0x89, 0x69, 0x80, 0x92, 0x02, 0x80, 0x91, 0x6a, 0x80, 0x87, 0x6a, 0x80, 0x92, 
	0x02, 0x80, 0x92, 0x6a, 0x80, 0x85, 0x6a, 0x80, 0x93, 0x02, 0x80, 0x93, 0x6a, 0x80, 0x83, 0x6a, 
	0x80, 0x94, 0x02, 0x80, 0x94, 0x6a, 0x37, 0x12, 0x38, 0x6a, 0x80, 0x95, 0x02, 0x80, 0x94, 0x6b, 
	0x30, 0x1f, 0x30, 0x6b, 0x80, 0x95, 0x02, 0x80, 0x95, 0x6b, 0x2b, 0x27, 0x2b, 0x6b, 0x80, 0x96, 
	0x02, 0x80, 0x96, 0x6c, 0x25, 0x2f, 0x25, 0x6c, 0x80, 0x97, 0x02, 0x80, 0x97, 0x6c, 0x21, 0x35, 
	0x21, 0x6c, 0x80, 0x98, 0x02, 0x80, 0x98, 0x6c, 0x1d, 0x3b, 0x1d, 0x6c, 0x80, 0x99, 0x02, 0x80, 
	0x99, 0x6c, 0x1a, 0x3f, 0x1a, 0x6c, 0x80, 0x9a, 0x02, 0x80, 0x9a, 0x6c, 0x17, 0x43, 0x17, 0x6c, 
	0x80, 0x9b, 0x02, 0x80, 0x9c, 0x6b, 0x14, 0x47, 0x14, 0x6b, 0x80, 0x9d, 0x02, 0x80, 0x9d, 0x6b, 
	0x11, 0x4b, 0x11, 0x6b, 0x80, 0x9e, 0x02, 0x80, 0x9f, 0x6a, 0x0e, 0x4f, 0x0e, 0x6a, 0x80, 0xa0, 
	0x02, 0x80, 0xa0, 0x68, 0x0d, 0x53, 0x0d, 0x68, 0x80, 0xa1, 0x02, 0x80, 0xa2, 0x64, 0x0e, 0x55, 
	0x0e, 0x64, 0x80, 0xa3, 0x02, 0x80, 0xa4, 0x61, 0x0d, 0x59, 0x0d, 0x61, 0x80, 0xa5, 0x02, 0x80, 
	0xa6, 0x5e, 0x0d, 0x5b, 0x0d, 0x5d, 0x80, 0xa8, 0x02, 0x80, 0xa9, 0x5a, 0x0c, 0x5f, 0x0c, 0x5a, 
	0x80, 0xaa, 0x02, 0x80, 0xad, 0x15, 0x04, 0x3c, 0x0c, 0x61, 0x0c, 0x3c, 0x04, 0x15, 0x80, 0xae, 
	0x02, 0x80, 0xb2, 0x0b, 0x0b, 0x39, 0x0c, 0x63, 0x0c, 0x39, 0x0b, 0x0b, 0x80, 0xb3, 0x02, 0x80, 
	0xc9, 0x37, 0x0b, 0x67, 0x0b, 0x37, 0x80, 0xca, 0x02, 0x80, 0xca, 0x35, 0x0b, 0x69, 0x0b, 0x35, 
	0x80, 0xcb, 0x02, 0x80, 0xcb, 0x33, 0x0b, 0x6b, 0x0b, 0x33, 0x80, 0xcc, 0x02, 0x80, 0xcc, 0x31, 
	0x0b, 0x6d, 0x0b, 0x31, 0x80, 0xcd, 0x02, 0x80, 0xcd, 0x2f, 0x0b, 0x6f, 0x0b, 0x2f, 0x80, 0xce, 
	0x02, 0x80, 0xce, 0x2d, 0x0b, 0x71, 0x0b, 0x2d, 0x80, 0xcf, 0x02, 0x80, 0xd0, 0x2a, 0x0b, 0x73, 
	0x0b, 0x2a, 0x80, 0xd1, 0x02, 0x80, 0xd1, 0x28, 0x0b, 0x75, 0x0b, 0x28, 0x80, 0xd2, 0x02, 0x80, 
	0xd2, 0x27, 0x0a, 0x77, 0x0a, 0x27, 0x80, 0xd3, 0x02, 0x80, 0xd3, 0x25, 0x0a, 0x79, 0x0a, 0x25, 
	0x80, 0xd4, 0x02, 0x80, 0xd4, 0x23, 0x0a, 0x7b, 0x0a, 0x23, 0x80, 0xd5, 0x02, 0x80, 0xd5, 0x21, 
	0x0a, 0x7d, 0x0a, 0x21, 0x80, 0xd6, 0x02, 0x80, 0xd6, 0x20, 0x09, 0x7f, 0x09, 0x20, 0x80, 0xd7, 
	0x02, 0x80, 0xd7, 0x1e, 0x09, 0x80, 0x81, 0x09, 0x1e, 0x80, 0xd8, 0x02, 0x80, 0xd9, 0x1b, 0x0a, 
	0x80, 0x81, 0x0a, 0x1b, 0x80, 0xda, 0x02, 0x80, 0xda, 0x19, 0x0a, 0x80, 0x83, 0x0a, 0x19, 0x80, 
	0xdb, 0x02, 0x80, 0xdb, 0x18, 0x09, 0x80, 0x85, 0x09, 0x18, 0x80, 0xdc, 0x02, 0x80, 0xdc, 0x16, 
	0x09, 0x80, 0x87, 0x09, 0x16, 0x80, 0xdd, 0x02, 0x80, 0xdd, 0x14, 0x0a, 0x80, 0x87, 0x0a, 0x14, 
	0x80, 0xde, 0x02, 0x80, 0xde, 0x13, 0x09, 0x80, 0x89, 0x09, 0x13, 0x80, 0xdf, 0x02, 0x80, 0xdf, 
	0x11, 0x09, 0x80, 0x8b, 0x09, 0x11, 0x80, 0xe0, 0x02, 0x80, 0xe0, 0x10, 0x09, 0x80, 0x8b, 0x09, 
	0x10, 0x80, 0xe1, 0x02, 0x80, 0xe2, 0x0d, 0x09, 0x80, 0x8d, 0x09, 0x0d, 0x80, 0xe3, 0x02, 0x80, 
	0xe3, 0x0b, 0x09, 0x80, 0x8f, 0x09, 0x0b, 0x80, 0xe4, 0x02, 0x80, 0xe4, 0x0a, 0x09, 0x80, 0x8f, 
	0x09, 0x0a, 0x80, 0xe5, 0x02, 0x80, 0xe5, 0x08, 0x09, 0x80, 0x91, 0x09, 0x08, 0x80, 0xe6, 0x02, 
	0x80, 0xe6, 0x07, 0x08, 0x80, 0x92, 0x09, 0x07, 0x80, 0xe7, 0x02, 0x80, 0xe7, 0x05, 0x09, 0x80, 
	0x93, 0x09, 0x05, 0x80, 0xe8, 0x02, 0x80, 0xe8, 0x04, 0x08, 0x80, 0x95, 0x08, 0x04, 0x80, 0xe9, 
	0x02, 0x80, 0xe9, 0x02, 0x09, 0x80, 0x95, 0x09, 0x01, 0x80, 0xeb, 0x02, 0x80, 0xf3, 0x80, 0x97, 
	0x80, 0xf4, 0x02, 0x80, 0xf3, 0x80, 0x97, 0x80, 0xf4, 0x02, 0x80, 0xf2, 0x80, 0x99, 0x80, 0xf3, 
	0x02, 0x80, 0xf2, 0x80, 0x99, 0x80, 0xf3, 0x02, 0x80, 0xf1, 0x80, 0x9b, 0x80, 0xf2, 0x02, 0x80, 
	0xf1, 0x80, 0x9b, 0x80, 0xf2, 0x02, 0x80, 0xf1, 0x80, 0x9b, 0x80, 0xf2, 0x02, 0x80, 0xf0, 0x80, 
	0x9d, 0x80, 0xf1, 0x02, 0x80, 0xf0, 0x80, 0x9d, 0x80, 0xf1, 0x02, 0x80, 0xef, 0x80, 0x9f, 0x80, 
	0xf0, 0x02, 0x80, 0xef, 0x80, 0x9f, 0x80, 0xf0, 0x02, 0x80, 0xef, 0x80, 0x9f, 0x80, 0xf0, 0x02, 
	0x80, 0xee, 0x80, 0xa1, 0x80, 0xef, 0x02, 0x80, 0xee, 0x80, 0xa1, 0x80, 0xef, 0x02, 0x80, 0xee, 
	0x80, 0xa1, 0x80, 0xef, 0x02, 0x80, 0xed, 0x80, 0xa3, 0x80, 0xee, 0x02, 0x80, 0xed, 0x80, 0xa3, 
	0x80, 0xee, 0x02, 0x80, 0xed, 0x80, 0xa3, 0x80, 0xee, 0x02, 0x80, 0xec, 0x80, 0xa5, 0x80, 0xed, 
	0x02, 0x80, 0xec, 0x80, 0xa5, 0x80, 0xed, 0x02, 0x80, 0xec, 0x80, 0xa5, 0x80, 0xed, 0x02, 0x80, 
	0xeb, 0x80, 0xa7, 0x80, 0xec, 0x02, 0x80, 0xeb, 0x80, 0xa7, 0x80, 0xec, 0x02, 0x80, 0xeb, 0x2b, 
	0x06, 0x45, 0x06, 0x2b, 0x80, 0xec, 0x02, 0x80, 0xeb, 0x26, 0x10, 0x3b, 0x10, 0x26, 0x80, 0xec, 
	0x02, 0x80, 0xeb, 0x24, 0x15, 0x35, 0x15, 0x24, 0x80, 0xec, 0x02, 0x80, 0xea, 0x23, 0x19, 0x31, 
	0x19, 0x23, 0x80, 0xeb, 0x02, 0x80, 0xea, 0x22, 0x1c, 0x2d, 0x1c, 0x22, 0x80, 0xeb, 0x02, 0x80, 
	0xea, 0x20, 0x1f, 0x2b, 0x1f, 0x20, 0x80, 0xeb, 0x02, 0x80, 0xea, 0x1f, 0x21, 0x29, 0x21, 0x1f, 
	0x80, 0xeb, 0x02, 0x80, 0xea, 0x1e, 0x23, 0x27, 0x23, 0x1e, 0x80, 0xeb, 0x02, 0x80, 0xe9, 0x1e, 
	0x25, 0x25, 0x25, 0x1e, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x1e, 0x26, 0x23, 0x26, 0x1e, 0x80, 0xea, 
	0x02, 0x80, 0xe9, 0x1d, 0x27, 0x23, 0x27, 0x1d, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x1c, 0x29, 0x21, 
	0x29, 0x1c, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x1b, 0x2b, 0x1f, 0x2b, 0x1b, 0x80, 0xea, 0x02, 0x80, 
	0xe9, 0x1b, 0x2b, 0x1f, 0x2b, 0x1b, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x1a, 0x2d, 0x1d, 0x2d, 0x1a, 
	0x80, 0xea, 0x02, 0x80, 0xe9, 0x1a, 0x2d, 0x1d, 0x2d, 0x1a, 0x80, 0xea, 0x02, 0x80, 0xe8, 0x1a, 
	0x2e, 0x1d, 0x2e, 0x1a, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x1a, 0x2e, 0x1d, 0x2e, 0x1a, 0x80, 0xe9, 
	0x02, 0x80, 0xe8, 0x19, 0x2f, 0x1d, 0x2f, 0x19, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x19, 0x2f, 0x1d, 
	0x2f, 0x19, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x19, 0x2f, 0x1d, 0x2f, 0x19, 0x80, 0xe9, 0x02, 0x80, 
	0xe8, 0x19, 0x2e, 0x1f, 0x2e, 0x19, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x2f, 0x1f, 0x2f, 0x18, 
	0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x2e, 0x21, 0x2e, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 
	0x2e, 0x21, 0x2e, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x2d, 0x23, 0x2d, 0x18, 0x80, 0xe9, 
	0x02, 0x80, 0xe8, 0x18, 0x2c, 0x25, 0x2c, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x2b, 0x27, 
	0x2b, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x2a, 0x29, 0x2a, 0x18, 0x80, 0xe9, 0x02, 0x80, 
	0xe8, 0x18, 0x28, 0x2d, 0x28, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x27, 0x2f, 0x27, 0x18, 
	0x80, 0xe9, 0x02, 0x80, 0xe8, 0x18, 0x25, 0x33, 0x25, 0x18, 0x80, 0xe9, 0x02, 0x80, 0xe8, 0x19, 
	0x21, 0x39, 0x21, 0x19, 0x80, 0xe9, 0x02, 0x80, 0xe9, 0x18, 0x1f, 0x3d, 0x1f, 0x18, 0x80, 0xea, 
	0x02, 0x80, 0xe9, 0x18, 0x1c, 0x43, 0x1c, 0x18, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x18, 0x1a, 0x47, 
	0x1a, 0x18, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x18, 0x18, 0x4b, 0x18, 0x18, 0x80, 0xea, 0x02, 0x80, 
	0xe9, 0x19, 0x15, 0x4f, 0x15, 0x19, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x19, 0x13, 0x53, 0x13, 0x19, 
	0x80, 0xea, 0x02, 0x80, 0xe9, 0x19, 0x12, 0x55, 0x12, 0x19, 0x80, 0xea, 0x02, 0x80, 0xe9, 0x1a, 
	0x10, 0x57, 0x10, 0x1a, 0x80, 0xea, 0x02, 0x80, 0xea, 0x19, 0x0f, 0x59, 0x0f, 0x19, 0x80, 0xeb, 
	0x02, 0x80, 0xea, 0x1a, 0x0e, 0x59, 0x0e, 0x1a, 0x80, 0xeb, 0x02, 0x80, 0xea, 0x1b, 0x0d, 0x59, 
	0x0d, 0x1b, 0x80, 0xeb, 0x02, 0x80, 0xea, 0x1b, 0x0c, 0x5b, 0x0c, 0x1b, 0x80, 0xeb, 0x02, 0x80, 
	0xea, 0x1c, 0x0b, 0x5b, 0x0b, 0x1c, 0x80, 0xeb, 0x02, 0x80, 0xeb, 0x1c, 0x0a, 0x5b, 0x0a, 0x1c, 
	0x80, 0xec, 0x02, 0x80, 0xeb, 0x1e, 0x08, 0x5b, 0x08, 0x1e, 0x80, 0xec, 0x02, 0x80, 0xeb, 0x1f, 
	0x07, 0x5b, 0x07, 0x1f, 0x80, 0xec, 0x02, 0x80, 0xeb, 0x20, 0x05, 0x2e, 0x01, 0x2e, 0x05, 0x20, 
	0x80, 0xec, 0x02, 0x80, 0xeb, 0x22, 0x03, 0x2d, 0x04, 0x2c, 0x03, 0x21, 0x80, 0xed, 0x02, 0x80, 
	0xec, 0x50, 0x06, 0x4f, 0x80, 0xed, 0x02, 0x80, 0xec, 0x4f, 0x07, 0x4f, 0x80, 0xed, 0x02, 0x80, 
	0xec, 0x4f, 0x08, 0x4e, 0x80, 0xed, 0x02, 0x80, 0xed, 0x4d, 0x09, 0x4d, 0x80, 0xee, 0x02, 0x80, 
	0xed, 0x4d, 0x09, 0x4d, 0x80, 0xee, 0x02, 0x80, 0xed, 0x4d, 0x0a, 0x4c, 0x80, 0xee, 0x02, 0x80, 
	0xee, 0x4b, 0x0b, 0x4b, 0x80, 0xef, 0x02, 0x80, 0xee, 0x4b, 0x0c, 0x4a, 0x80, 0xef, 0x02, 0x80, 
	0xee, 0x4b, 0x0c, 0x4a, 0x80, 0xef, 0x02, 0x80, 0xef, 0x49, 0x0d, 0x49, 0x80, 0xf0, 0x02, 0x80, 
	0xef, 0x49, 0x0d, 0x49, 0x80, 0xf0, 0x02, 0x80, 0xef, 0x49, 0x0e, 0x48, 0x80, 0xf0, 0x02, 0x80, 
	0xf0, 0x48, 0x0e, 0x47, 0x80, 0xf1, 0x02, 0x80, 0xe6, 0x02, 0x08, 0x47, 0x0f, 0x47, 0x08, 0x02, 
	0x80, 0xe7, 0x02, 0x80, 0xe5, 0x04, 0x08, 0x46, 0x0f, 0x46, 0x08, 0x04, 0x80, 0xe6, 0x02, 0x80, 
	0xe4, 0x05, 0x08, 0x46, 0x0f, 0x46, 0x08, 0x05, 0x80, 0xe5, 0x02, 0x80, 0xe3, 0x06, 0x08, 0x46, 
	0x0f, 0x45, 0x09, 0x06, 0x80, 0xe4, 0x02, 0x80, 0xe2, 0x08, 0x08, 0x45, 0x10, 0x44, 0x08, 0x08, 
	0x80, 0xe3, 0x02, 0x80, 0xe1, 0x09, 0x08, 0x44, 0x07, 0x03, 0x07, 0x44, 0x08, 0x09, 0x80, 0xe2, 
	0x02, 0x80, 0xe0, 0x0b, 0x08, 0x43, 0x06, 0x05, 0x06, 0x43, 0x08, 0x0b, 0x80, 0xe1, 0x02, 0x80, 
	0xde, 0x0d, 0x08, 0x43, 0x05, 0x07, 0x05, 0x43, 0x08, 0x0d, 0x80, 0xdf, 0x02, 0x80, 0xdd, 0x0f, 
	0x08, 0x42, 0x05, 0x07, 0x05, 0x42, 0x08, 0x0f, 0x80, 0xde, 0x02, 0x80, 0xdc, 0x10, 0x08, 0x42, 
	0x05, 0x07, 0x05, 0x42, 0x08, 0x10, 0x80, 0xdd, 0x02, 0x80, 0xdb, 0x12, 0x08, 0x41, 0x04, 0x09, 
	0x04, 0x41, 0x08, 0x12, 0x80, 0xdc, 0x02, 0x80, 0xda, 0x13, 0x09, 0x40, 0x04, 0x09, 0x04, 0x40, 
	0x09, 0x13, 0x80, 0xdb, 0x02, 0x80, 0xd9, 0x15, 0x08, 0x40, 0x03, 0x0a, 0x04, 0x40, 0x08, 0x15, 
	0x80, 0xda, 0x02, 0x80, 0xd8, 0x16, 0x09, 0x3f, 0x03, 0x0a, 0x03, 0x40, 0x09, 0x16, 0x80, 0xd9, 
	0x02, 0x80, 0xd7, 0x18, 0x08, 0x40, 0x02, 0x0b, 0x02, 0x40, 0x08, 0x18, 0x80, 0xd8, 0x02, 0x80, 
	0xd5, 0x1a, 0x09, 0x4c, 0x01, 0x40, 0x09, 0x1a, 0x80, 0xd6, 0x02, 0x80, 0xd4, 0x1c, 0x09, 0x80, 
	0x8b, 0x09, 0x1c, 0x80, 0xd5, 0x02, 0x80, 0xd3, 0x1d, 0x09, 0x80, 0x8b, 0x09, 0x1d, 0x80, 0xd4, 
	0x02, 0x80, 0xd2, 0x1f, 0x09, 0x80, 0x89, 0x09, 0x1f, 0x80, 0xd3, 0x02, 0x80, 0xd1, 0x21, 0x09, 
	0x80, 0x87, 0x09, 0x21, 0x80, 0xd2, 0x02, 0x80, 0xd0, 0x22, 0x09, 0x80, 0x87, 0x09, 0x22, 0x80, 
	0xd1, 0x02, 0x80, 0xcf, 0x24, 0x09, 0x80, 0x85, 0x09, 0x24, 0x80, 0xd0, 0x02, 0x80, 0xcd, 0x27, 
	0x09, 0x80, 0x83, 0x09, 0x27, 0x80, 0xce, 0x02, 0x80, 0xcc, 0x28, 0x0a, 0x80, 0x81, 0x0a, 0x28, 
	0x80, 0xcd, 0x02, 0x80, 0xcb, 0x2a, 0x0a, 0x7f, 0x0a, 0x2a, 0x80, 0xcc, 0x02, 0x80, 0xca, 0x2c, 
	0x09, 0x7f, 0x09, 0x2c, 0x80, 0xcb, 0x02, 0x80, 0xc9, 0x2e, 0x09, 0x7d, 0x09, 0x2e, 0x80, 0xca, 
	0x02, 0x80, 0xc8, 0x2f, 0x0a, 0x7b, 0x0a, 0x2f, 0x80, 0xc9, 0x02, 0x80, 0xb4, 0x07, 0x0c, 0x31, 
	0x0a, 0x79, 0x0a, 0x31, 0x0c, 0x07, 0x80, 0xb5, 0x02, 0x80, 0xad, 0x15, 0x04, 0x33, 0x0a, 0x77, 
	0x0a, 0x33, 0x04, 0x15, 0x80, 0xae, 0x02, 0x80, 0xa9, 0x51, 0x0a, 0x75, 0x0a, 0x51, 0x80, 0xaa, 
	0x02, 0x80, 0xa7, 0x54, 0x0a, 0x73, 0x0a, 0x54, 0x80, 0xa8, 0x02, 0x80, 0xa4, 0x58, 0x0a, 0x71, 
	0x0a, 0x58, 0x80, 0xa5, 0x02, 0x80, 0xa2, 0x5a, 0x0b, 0x6f, 0x0b, 0x5a, 0x80, 0xa3, 0x02, 0x80, 
	0xa0, 0x5d, 0x0b, 0x6d, 0x0b, 0x5d, 0x80, 0xa1, 0x02, 0x80, 0x9f, 0x5f, 0x0b, 0x6b, 0x0b, 0x5f, 
	0x80, 0xa0, 0x02, 0x80, 0x9d, 0x62, 0x0b, 0x69, 0x0b, 0x62, 0x80, 0x9e, 0x02, 0x80, 0x9c, 0x64, 
	0x0c, 0x65, 0x0c, 0x64, 0x80, 0x9d, 0x02, 0x80, 0x9b, 0x66, 0x0c, 0x63, 0x0c, 0x66, 0x80, 0x9c, 
	0x02, 0x80, 0x99, 0x69, 0x0b, 0x62, 0x0c, 0x69, 0x80, 0x9a, 0x02, 0x80, 0x98, 0x6b, 0x0a, 0x63, 
	0x0a, 0x6b, 0x80, 0x99, 0x02, 0x80, 0x97, 0x6b, 0x0a, 0x64, 0x0b, 0x6b, 0x80, 0x98, 0x02, 0x80, 
	0x96, 0x6a, 0x0c, 0x65, 0x0c, 0x6a, 0x80, 0x97, 0x02, 0x80, 0x95, 0x6a, 0x0d, 0x65, 0x0d, 0x6a, 
	0x80, 0x96, 0x02, 0x80, 0x94, 0x6a, 0x0e, 0x65, 0x0e, 0x6a, 0x80, 0x95, 0x02, 0x80, 0x93, 0x6a, 
	0x0e, 0x66, 0x0f, 0x6a, 0x80, 0x94, 0x02, 0x80, 0x93, 0x69, 0x0f, 0x21, 0x02, 0x21, 0x02, 0x21, 
	0x0f, 0x69, 0x80, 0x94, 0x02, 0x80, 0x92, 0x69, 0x10, 0x21, 0x02, 0x21, 0x02, 0x21, 0x10, 0x69, 
	0x80, 0x93, 0x02, 0x80, 0x91, 0x69, 0x11, 0x21, 0x02, 0x21, 0x02, 0x21, 0x11, 0x69, 0x80, 0x92, 
	0x02, 0x80, 0x91, 0x68, 0x12, 0x21, 0x02, 0x21, 0x02, 0x21, 0x12, 0x68, 0x80, 0x92, 0x02, 0x80, 
	0x90, 0x67, 0x14, 0x21, 0x02, 0x21, 0x02, 0x21, 0x14, 0x67, 0x80, 0x91, 0x02, 0x80, 0x8f, 0x67, 
	0x16, 0x20, 0x03, 0x1f, 0x04, 0x1f, 0x16, 0x67, 0x80, 0x90, 0x02, 0x80, 0x8f, 0x66, 0x17, 0x20, 
	0x03, 0x1f, 0x04, 0x1f, 0x17, 0x66, 0x80, 0x90, 0x02, 0x80, 0x8e, 0x66, 0x18, 0x1f, 0x04, 0x1f, 
	0x04, 0x1f, 0x18, 0x66, 0x80, 0x8f, 0x02, 0x80, 0x8e, 0x65, 0x1a, 0x1e, 0x04, 0x1f, 0x04, 0x1f, 
	0x19, 0x65, 0x80, 0x8f, 0x02, 0x80, 0x8d, 0x65, 0x1b, 0x1d, 0x06, 0x1d, 0x06, 0x1d, 0x1b, 0x65, 
	0x80, 0x8e, 0x02, 0x80, 0x8d, 0x64, 0x1d, 0x1c, 0x06, 0x1d, 0x06, 0x1d, 0x1c, 0x64, 0x80, 0x8e, 
	0x02, 0x80, 0x8d, 0x1a, 0x02, 0x47, 0x1e, 0x1b, 0x08, 0x1b, 0x08, 0x1b, 0x1e, 0x47, 0x02, 0x1a, 
	0x80, 0x8e, 0x02, 0x80, 0x8c, 0x19, 0x05, 0x44, 0x21, 0x19, 0x0a, 0x19, 0x0a, 0x19, 0x21, 0x44, 
	0x05, 0x19, 0x80, 0x8d, 0x02, 0x80, 0x8c, 0x18, 0x07, 0x42, 0x23, 0x18, 0x0b, 0x17, 0x0c, 0x17, 
	0x23, 0x42, 0x07, 0x18, 0x80, 0x8d, 0x02, 0x80, 0x8b, 0x18, 0x09, 0x40, 0x25, 0x16, 0x0d, 0x15, 
	0x0e, 0x15, 0x25, 0x40, 0x09, 0x18, 0x80, 0x8c, 0x02, 0x80, 0x8b, 0x17, 0x0b, 0x3e, 0x27, 0x13, 
	0x10, 0x13, 0x10, 0x13, 0x27, 0x3e, 0x0b, 0x17, 0x80, 0x8c, 0x02, 0x80, 0x8b, 0x16, 0x0d, 0x3c, 
	0x2a, 0x10, 0x12, 0x11, 0x12, 0x11, 0x29, 0x3c, 0x0d, 0x16, 0x80, 0x8c, 0x02, 0x80, 0x8b, 0x15, 
	0x0f, 0x3a, 0x2d, 0x0c, 0x17, 0x0b, 0x18, 0x0b, 0x2d, 0x3a, 0x0f, 0x15, 0x80, 0x8c, 0x02, 0x80, 
	0x8a, 0x15, 0x11, 0x38, 0x32, 0x04, 0x1e, 0x05, 0x1e, 0x05, 0x31, 0x38, 0x11, 0x15, 0x80, 0x8b, 
	0x02, 0x80, 0x8a, 0x14, 0x12, 0x37, 0x80, 0xaf, 0x37, 0x12, 0x14, 0x80, 0x8b, 0x02, 0x80, 0x8a, 
	0x12, 0x15, 0x34, 0x80, 0xb3, 0x34, 0x15, 0x12, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x11, 0x17, 0x33, 
	0x80, 0xb3, 0x33, 0x17, 0x11, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x10, 0x19, 0x32, 0x80, 0xb3, 0x32, 
	0x19, 0x10, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x0f, 0x1b, 0x31, 0x80, 0xb3, 0x31, 0x1b, 0x0f, 0x80, 
	0x8b, 0x02, 0x80, 0x89, 0x0f, 0x1d, 0x31, 0x80, 0xb1, 0x31, 0x1d, 0x0f, 0x80, 0x8a, 0x02, 0x80, 
	0x89, 0x0e, 0x1f, 0x30, 0x80, 0xb1, 0x30, 0x1f, 0x0e, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0d, 0x21, 
	0x2f, 0x80, 0xb1, 0x2f, 0x21, 0x0d, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0c, 0x23, 0x2e, 0x80, 0xb1, 
	0x2e, 0x23, 0x0c, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x0a, 0x26, 0x2d, 0x80, 0xb1, 0x2d, 0x26, 0x0a, 
	0x80, 0x8a, 0x02, 0x80, 0x89, 0x09, 0x27, 0x2d, 0x80, 0xb1, 0x2d, 0x27, 0x09, 0x80, 0x8a, 0x02, 
	0x80, 0x89, 0x08, 0x29, 0x2c, 0x80, 0xb1, 0x2c, 0x29, 0x08, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x07, 
	0x2b, 0x2b, 0x80, 0xb1, 0x2b, 0x2b, 0x07, 0x80, 0x8a, 0x02, 0x80, 0x89, 0x06, 0x2d, 0x2a, 0x80, 
	0xb1, 0x2a, 0x2d, 0x06, 0x80, 0x8a, 0x02, 0x80, 0x8a, 0x04, 0x2f, 0x28, 0x80, 0xb3, 0x28, 0x2f, 
	0x04, 0x80, 0x8b, 0x02, 0x80, 0x8a, 0x03, 0x31, 0x27, 0x80, 0xb3, 0x27, 0x31, 0x03, 0x80, 0x8b, 
	0x02, 0x80, 0x8a, 0x02, 0x33, 0x26, 0x80, 0xb3, 0x26, 0x33, 0x02, 0x80, 0x8b, 0x02, 0x80, 0xc0, 
	0x25, 0x80, 0xb3, 0x25, 0x80, 0xc1, 0x02, 0x80, 0xc1, 0x24, 0x80, 0xb3, 0x24, 0x80, 0xc2, 0x02, 
	0x80, 0xc2, 0x23, 0x80, 0xb3, 0x23, 0x80, 0xc3, 0x02, 0x80, 0xc3, 0x21, 0x80, 0xb5, 0x21, 0x80, 
	0xc4, 0x02, 0x80, 0xc3, 0x21, 0x80, 0xb5, 0x21, 0x80, 0xc4, 0x02, 0x80, 0xc4, 0x20, 0x80, 0xb5, 
	0x20, 0x80, 0xc5, 0x02, 0x80, 0xc5, 0x1e, 0x80, 0xb7, 0x1e, 0x80, 0xc6, 0x02, 0x80, 0xc6, 0x1d, 
	0x80, 0xb7, 0x1d, 0x80, 0xc7, 0x02, 0x80, 0xc7, 0x1c, 0x80, 0xb7, 0x1c, 0x80, 0xc8, 0x02, 0x80, 
	0xc8, 0x1a, 0x80, 0xb9, 0x1a, 0x80, 0xc9, 0x02, 0x80, 0xc8, 0x1a, 0x80, 0xb9, 0x1a, 0x80, 0xc9, 
	0x02, 0x80, 0xc7, 0x1b, 0x80, 0xb9, 0x1b, 0x80, 0xc8, 0x02, 0x80, 0xc5, 0x1c, 0x80, 0xbb, 0x1c, 
	0x80, 0xc6, 0x02, 0x80, 0xc4, 0x1d, 0x80, 0xbb, 0x1d, 0x80, 0xc5, 0x02, 0x80, 0xc3, 0x1d, 0x80, 
	0xbd, 0x1d, 0x80, 0xc4, 0x02, 0x80, 0xc2, 0x1d, 0x80, 0xbf, 0x1d, 0x80, 0xc3, 0x02, 0x80, 0xc1, 
	0x1e, 0x80, 0xbf, 0x1e, 0x80, 0xc2, 0x02, 0x80, 0xc0, 0x1e, 0x80, 0xc1, 0x1e, 0x80, 0xc1, 0x02, 
	0x80, 0xbf, 0x1f, 0x80, 0xc1, 0x1f, 0x80, 0xc0, 0x02, 0x80, 0xbe, 0x1f, 0x80, 0xc3, 0x1f, 0x80, 
	0xbf, 0x02, 0x80, 0xbc, 0x20, 0x80, 0xc5, 0x20, 0x80, 0xbd, 0x02, 0x80, 0xbb, 0x20, 0x80, 0xc7, 
	0x20, 0x80, 0xbc, 0x02, 0x80, 0xba, 0x20, 0x80, 0xc9, 0x20, 0x80, 0xbb, 0x02, 0x80, 0xb9, 0x21, 
	0x80, 0xc9, 0x21, 0x80, 0xba, 0x02, 0x80, 0xb8, 0x21, 0x80, 0xcb, 0x21, 0x80, 0xb9, 0x02, 0x80, 
	0xb7, 0x21, 0x80, 0xcd, 0x21, 0x80, 0xb8, 0x02, 0x80, 0xb6, 0x21, 0x80, 0xcf, 0x21, 0x80, 0xb7, 
	0x02, 0x80, 0xb5, 0x20, 0x80, 0xd3, 0x20, 0x80, 0xb6, 0x02, 0x80, 0xb3, 0x21, 0x80, 0xd5, 0x21, 
	0x80, 0xb4, 0x02, 0x80, 0xb2, 0x21, 0x80, 0xd7, 0x21, 0x80, 0xb3, 0x02, 0x80, 0xb1, 0x21, 0x80, 
	0xd9, 0x21, 0x80, 0xb2, 0x02, 0x80, 0xb0, 0x20, 0x80, 0xdd, 0x20, 0x80, 0xb1, 0x02, 0x80, 0xaf, 
	0x1f, 0x80, 0xe1, 0x1f, 0x80, 0xb0, 0x02, 0x80, 0xae, 0x1f, 0x80, 0xe3, 0x1f, 0x80, 0xaf, 0x02, 
	0x80, 0xad, 0x1d, 0x80, 0xe8, 0x1e, 0x80, 0xae, 0x02, 0x80, 0xac, 0x1c, 0x80, 0xed, 0x1c, 0x80, 
	0xad, 0x02, 0x80, 0xab, 0x1a, 0x80, 0xf3, 0x1a, 0x80, 0xac, 0x02, 0x80, 0xae, 0x13, 0x80, 0xfb, 
	0x13, 0x80, 0xaf, 0x02, 0x80, 0xb4, 0x07, 0x81, 0x07, 0x07, 0x80, 0xb5, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 
	0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 
	0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 
	0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x02, 0x82, 0x7e, 0x82, 0x81, 
};

// Compressed RLE size 3900

void drawRLE()
{
	// pseudo-code for cheesy RLE
	// start with color1
	// while more input data remaining
	// 	count =  0nnnnnnn = 1 byte or 1nnnnnnn nnnnnnnn 2 bytes (0 - 32767)
	// 	repeat color count times
	// 	toggle color1/color2
	
	uint32_t	cnt = 0;
	int			color = 1;
	int			curcolor = 0;
	int			x = 0, y = 0;

	const uint8_t *cmp = &HaDLogo[0];

	while (cmp < &HaDLogo[sizeof(HaDLogo)])
	{
		cnt = *cmp++;
		if (cnt & 0x80)
			cnt = ((cnt & 0x7f) << 8) | *cmp++;

		while (cnt--)
		{
			plot(x, y, curcolor ? (((x>>1)+(y))) & 0xff : 0x00);
				
			if (++x >= 640)
			{
				x = 0;
				if (++y >= 480)
					y = 0;
			}
		}

		curcolor ^= color;
	}
}
