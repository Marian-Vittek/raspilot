from tentacle_pi.BMP180 import BMP180
import time
bmp = BMP180(0x77,"/dev/i2c-1")


for x in range(0,100):
        print "temperature: %0.1f" % bmp.temperature()
        print "pressure: %s" % bmp.pressure()
        print "altitude: %0.1f" % bmp.altitude()
        print
        time.sleep(2)

