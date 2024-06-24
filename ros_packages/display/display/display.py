from abc import ABC
from hashlib import sha256
from queue import Queue
import base64
from dataclasses import dataclass
from functools import reduce
from io import BytesIO
from itertools import cycle
import os
from threading import Thread
from typing import Iterable, Iterator, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import PIL.Image

from tkinter import *

from datatypes.msg import DisplayImage, ImageFormat, ImageId

import os


# this env-variable specifies the display that
# x-server should use. If none is specified,
# use ':0.0' as default
os.environ.setdefault("DISPLAY", ":0.0")

# points to the directory, where all static images are
# stored that are managed by the display-node
STATIC_IMAGE_DIR: str = os.getenv(
    "STATIC_IMAGE_DIR",
    "/home/pib/ros_working_dir/src/display/static_images",
)


@dataclass
class SerializedImage:
    """represents an image stored in the filesystem"""
    format: bytes
    filepath: str


@dataclass
class DeserializedImage:
    """an image whose data was loaded into main-memory"""
    format: bytes
    data: bytes


    @staticmethod
    def from_display_image(display_image: DisplayImage):
        """turns a DisplayImaGE into a DeserializedImage"""
        image_id = display_image.id.value
        match image_id:
            case ImageId.CUSTOM:
                return DeserializedImage(
                    display_image.format.value,
                    b''.join(display_image.data)
                )
            case ImageId.NONE:
                return None
            case _:
                serialized_image = IMAGE_ID_TO_STATIC_IMAGES.get(image_id)
                if serialized_image is None:
                    raise Exception(f"illegal image-id: '{image_id}'.")
                return DeserializedImage.from_serialized_image(serialized_image)
            
    
    @staticmethod
    def from_serialized_image(serialized_image: SerializedImage):
        """turns a SerializedImage into a DeserializedImage"""
        with open(serialized_image.filepath, "rb") as file:
            data = file.read()
        return DeserializedImage(serialized_image.format, data)
    

@dataclass
class AnimationFrame:
    """represents one frame of an animated gif"""
    duration_ms: int
    photo_image: PhotoImage


class Animation:
    """can be used to iterate over the frames of an animated gif"""

    def __init__(self, data: bytes, width: int, height: int):
        """initalizes the animation, with the given image-data"""
        self._frames: Iterable[AnimationFrame] = self._as_frames(data, width, height)
        self._frame_iterator = iter(cycle(self._frames))
        self._stopped = False
    
    def stop(self) -> None:
        """stop the iterator"""
        self._stopped = True

    def __iter__(self) -> Iterator[AnimationFrame]:
        return self

    def __next__(self) -> AnimationFrame:
        if self._stopped:
            raise StopIteration()
        else:
            return next(self._frame_iterator) 
        
    def _as_frames(self, data: bytes, width: int, height: int) -> Iterable[AnimationFrame]:
        with PIL.Image.open(BytesIO(data)) as image:
            n = image.n_frames
        queue = Queue()
        def f(data):
            with PIL.Image.open(BytesIO(data)) as image:
                for i in range(image.n_frames): # iterate over frames of image
                    image.seek(i) # go to i-th frame of the image
                    data_buffer = BytesIO() # buffer for storing binary data of image-frames
                    resized = (
                        image.resize((width, height))
                        if image.width != width or image.height != height
                        else image
                    )
                    resized.save(data_buffer, "gif") # save the current frame in the data-buffer
                    data = base64.b64encode(data_buffer.getvalue()) # extract and encode data in data-buffer
                    # photo_image = PhotoImage(data=data) # image, displaying the current frame
                    duration_ms = image.info["duration"] # get the duration of the current frame
                    queue.put((data, duration_ms))  # yield the extracted data
                    data_buffer.flush() # clear the data-buffer
        Thread(target=f, args=(data,)).start()
        for _ in range(n):
            data, duration_ms = queue.get()
            yield AnimationFrame(duration_ms, PhotoImage(data=data))
                

# maps an image-id to its corresponding image in the filesystem
IMAGE_ID_TO_STATIC_IMAGES: dict[int, SerializedImage] = {
    ImageId.PIB_EYES_ANIMATED: SerializedImage(
        ImageFormat.ANIMATED_GIF, 
        STATIC_IMAGE_DIR + "/pib-eyes-animated.gif"
    ),
}

FORMAT_ID_TO_STR: dict[int, str] = {
    ImageFormat.ANIMATED_GIF: "gif",
    ImageFormat.PNG: "png",
    ImageFormat.JPEG: "jpg",
}


class Application(Frame):
    
    def __init__(self, parent: Widget, image_queue: Queue[DeserializedImage | None], inital_image: DeserializedImage, *args, **kwargs):

        # call to the constructor of the superclass
        Frame.__init__(self, parent, *args, **kwargs)

        # store the image_queue to poll from it for images to show
        self.image_queue = image_queue

        # store the width and height of the screen
        # this widget together with all the images that it
        # displays, will be scaled to the screen-size
        self._width = parent.winfo_screenwidth()
        self._height = parent.winfo_screenheight()

        # define a canvas where the main-image is displayed
        self.canvas = Canvas(
            parent, 
            width=self._width,
            height=self._height,
            borderwidth=0, 
            highlightthickness=0
        )
        self.canvas.place(x=0, y=0)

        # current static-image or animated-gif are stored here
        self.current_main_content: PhotoImage | Animation | None = None

        # intiate periodically polling for images in the queue
        self._poll_next_image()

        self._show_image(inital_image)

        # position the widget in its parent
        self.grid()

    def _show_image(self, deserialized_image: DeserializedImage) -> None:
        """update the main background image"""
        self.canvas.delete("all")
        if isinstance(self.current_main_content, Animation):
            self.current_main_content.stop()
        if deserialized_image.format == ImageFormat.ANIMATED_GIF:
            self._show_animated_gif(deserialized_image)
        else:
            self._show_static_image(deserialized_image)

    def _show_animated_gif(self, deserialized_image: DeserializedImage) -> None:
        self.current_main_content = Animation(
            deserialized_image.data, 
            self._width, 
            self._height
        )
        self._show_next_frame()

    def _show_static_image(self, deserialized_image: DeserializedImage) -> None:
        format = FORMAT_ID_TO_STR[deserialized_image.format]
        with PIL.Image.open(BytesIO(deserialized_image.data)) as image:
            resized = image.resize((self._width, self._height))
            data_buffer = BytesIO() # buffer for storing binary data of image-frame
            resized.save(data_buffer, "png") # save the current frame in the data-buffer
            data = base64.b64encode(data_buffer.getvalue())
        self.current_main_content = PhotoImage(data=data, format=format)
        self.canvas.create_image(self.current_main_content)

    def _show_next_frame(self) -> None:
        try:
            frame: AnimationFrame = next(self.current_main_content)
        except StopIteration:
            return
        self.canvas.delete("all")
        self.canvas.create_image(0,0, image=frame.photo_image, anchor="nw")
        self.canvas.after(frame.duration_ms, self._show_next_frame)

    def _poll_next_image(self) -> None:
        if self.image_queue.qsize() != 0:
            image: Optional[DeserializedImage] = self.image_queue.get()
            if image is None:
                self.winfo_toplevel().destroy()
                return
            else:
                print("a")
                self._show_image(image)
                print("b")
        self.after(100, self._poll_next_image)


class DisplayNode(Node):

    def __init__(self, image_queue: Queue[DeserializedImage | None]) -> None:

        super().__init__("display")

        self.create_subscription(
            DisplayImage,
            "display_image",
            self.on_display_image_received,
            1
        )

        self.image_queue = image_queue

        pib_eyes_animated = DeserializedImage.from_serialized_image(
            IMAGE_ID_TO_STATIC_IMAGES[ImageId.PIB_EYES_ANIMATED]
        )
        self.image_queue.put(pib_eyes_animated)

        self.get_logger().info("Now Running DISPLAY")

    def on_display_image_received(self, display_image: DisplayImage):
        """callback function for the 'display_image'-topic subscriber"""
        try:
            deserialized_image = DeserializedImage.from_display_image(display_image)
            self.image_queue.put(deserialized_image)
        except Exception as e:
            self.get_logger().error(f"error while showing image from topic: {e}.")
        

def run_gui_application(image_queue: Queue[DeserializedImage | None]) -> None:
    while True:
        image = image_queue.get()
        if image is None:
            continue
        root = Tk()
        root.attributes("-fullscreen", True)
        application = Application(root, image_queue, image)
        root.mainloop()


def run_display_node(image_queue: Queue[DeserializedImage | None]) -> None:
    rclpy.init()
    executor = MultiThreadedExecutor(2)
    display_node = DisplayNode(image_queue)
    executor.add_node(display_node)
    executor.spin()
    display_node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:   
    image_queue: Queue[DeserializedImage | None] = Queue()
    t = Thread(daemon=True, target=run_display_node, args=(image_queue,))
    t.start()
    run_gui_application(image_queue)   


if __name__ == "__main__":
    main()
