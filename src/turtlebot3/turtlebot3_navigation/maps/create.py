from PIL import Image

width, height = 1200, 1200  # 60m x 60m at 0.05m/pix
img = Image.new('L', (width, height), 254)  # 254 = white = free space
img.save("map.pgm")
print("✅ Dummy map.pgm created!")