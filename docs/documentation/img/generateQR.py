import qrcode
import qrcode.image.svg

# define a method to choose which factory metho to use
# possible values 'basic' 'fragment' 'path'
method = "basic"
URLPAGE = "shorturl.at/celtz"
URLCODE = "shorturl.at/DF179"

if method == 'basic':
    # Simple factory, just a set of rects.
    factory = qrcode.image.svg.SvgImage
elif method == 'fragment':
    # Fragment factory (also just a set of rects)
    factory = qrcode.image.svg.SvgFragmentImage
elif method == 'path':
    # Combined path factory, fixes white space that may occur when zooming
    factory = qrcode.image.svg.SvgPathImage

# Set data to qrcode
imgCode = qrcode.make(URLCODE, image_factory = factory)
imgPage = qrcode.make(URLPAGE, image_factory = factory)

# Save svg file somewhere
imgCode.save("qrCode.svg")
imgPage.save("qrPage.svg")

# Create qr code instance
qr = qrcode.QRCode(
    version = 1,
    error_correction = qrcode.constants.ERROR_CORRECT_H,
    box_size = 10,
    border = 4,
)

# Add data
qr.add_data(URLCODE)
qr.make(fit=True)

# Create an image from the QR Code instance
imgCode = qr.make_image()
imgCode.save("qrCode.png")

# Add data
qr.add_data(URLPAGE)
qr.make(fit=True)

# Create an image from the QR Code instance
imgPage = qr.make_image()
imgPage.save("qrPage.png")
