import asyncio
import cv2 as cv
from bleak import BleakClient
import numpy as np

address = "38:81:D7:32:51:4F"
read_write_characteristic_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

map_width = 600
map_height = 600

img = np.full((map_width,map_height,3),0,np.uint8)

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def trilateration(base1, base2, base3, d1, d2, d3):
    A = 2 * (base2.x - base1.x)
    B = 2 * (base2.y - base1.y)
    C = d1 * d1 - d2 * d2 - base1.x * base1.x + base2.x * base2.x - base1.y * base1.y + base2.y * base2.y
    D = 2 * (base3.x - base2.x)
    E = 2 * (base3.y - base2.y)
    F = d2 * d2 - d3 * d3 - base2.x * base2.x + base3.x * base3.x - base2.y * base2.y + base3.y * base3.y
    
    x = (C * E - F * B) / (E * A - B * D)
    y = (C * D - A * F) / (B * D - A * E)
    
    return Vector(x, y)

base1 = Vector(x=0, y=0)
base2 = Vector(x=3, y=0)
base3 = Vector(x=0, y=3)

d0 = 0
d1 = 0
d2 = 0

async def read_ble_data(address, uuid): 
    async with BleakClient(address) as client:
        print(f"Connected: {client.is_connected}")
        services = await client.get_services()
        for service in services:
            for characteristic in service.characteristics:
                if characteristic.uuid == uuid:
                    while True:
                        if 'read' in characteristic.properties:
                            data_bt_R = await client.read_gatt_char(characteristic)
                            print(data_bt_R)
                            extracted_data = data_bt_R.decode("utf-8").strip().split('\n')
                            
                            if len(extracted_data) >= 3:  # Ensure at least three values are available
                                a0, a1, a2 = map(float, extracted_data[:3])
                                d0, d1, d2 = a0, a1, a2
                                tag_pos = trilateration(base1, base2, base3, d0, d1, d2)
                                # Now you can use a0, a1, a2 as needed
                                print(f"a0: {a0}, a1: {a1}, a2: {a2}")
                                img = np.full((map_width,map_height,3),0,np.uint8)
                                cv.circle(img, (0, 0), 20, (0,255,0), thickness = -1) #anchor 2
                                cv.circle(img, (600, 600), 20, (0,255,0), thickness = -1) #anchor 0
                                cv.circle(img, (0, 600), 20, (0,255,0), thickness = -1) #anchor 1
                                cv.line(img,(0,0),(int(float(tag_pos.x)*200),int(600-float(tag_pos.y)*200)),(0,255,0),2)
                                cv.line(img,(0,600),(int(float(tag_pos.x)*200),int(600-float(tag_pos.y)*200)),(0,255,0),2)
                                cv.line(img,(600,600),(int(float(tag_pos.x)*200),int(600-float(tag_pos.y)*200)),(0,255,0),2)
                                cv.putText(img, str(d2), (int(float(tag_pos.x)*100), int(300-float(tag_pos.y)*100)), cv.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                                cv.putText(img, str(d1), (int(float(tag_pos.x)*100), 300+int(300-float(tag_pos.y)*100)), cv.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                                cv.putText(img, str(d0), (300+int(float(tag_pos.x)*100), 300+int(300-float(tag_pos.y)*100)), cv.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                                x = round(float(tag_pos.x),2)
                                y = round(float(tag_pos.y),2)
                            
                                cv.circle(img, (int(float(tag_pos.x)*200), int(600-float(tag_pos.y)*200)), 20, (255,255,255), thickness = -1) #파란 영역
                                cv.putText(img, "x :"+str(x), (50, 50), cv.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                                cv.putText(img, "y :"+str(y), (50, 70), cv.FONT_ITALIC, 0.5, (255, 255, 255), 1)
                                #print(f"Tag Position: x={int(tag_pos.x)*200}, y={600-int(tag_pos.y)*200}")
                                cv.imshow('map', img)
                                
                        await asyncio.sleep(0.1)
                    cv.destroyAllWindows()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(read_ble_data(address, read_write_characteristic_uuid))
