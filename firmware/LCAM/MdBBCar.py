from maix import camera, time, app, http

html = """<!DOCTYPE html>
<html>
<head>
    <title>JPG Stream</title>
</head>
<body>
    <h1>MdBBCar JPG Stream</h1>
    <img src="/stream" alt="Stream">
</body>
</html>"""

cam = camera.Camera(640, 480)
stream = http.JpegStreamer()
stream.set_html(html)
stream.start()

print("http://{}:{}".format(stream.host(), stream.port()))
while not app.need_exit():
    t = time.time_ms()
    img = cam.read()
    stream.write(img)
    # print(f"time: {time.time_ms() - t}ms, fps: {1000 / (time.time_ms() - t)}")