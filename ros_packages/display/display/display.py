from queue import Queue
import base64
from dataclasses import dataclass
from io import BytesIO
from itertools import cycle
import os
from threading import Thread
from typing import Iterable, Iterator, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import PIL.Image

from tkinter import *

from datatypes.msg import DisplayImage, ImageFormat, ImageId

import os


os.environ.setdefault("DISPLAY", ":0.0")


# points to the directory, where all static images are
# stored that are managed by the display-node
STATIC_IMAGE_DIR: str = os.getenv(
    "STATIC_IMAGE_DIR",
    "/home/pib/ros_working_dir/src/display/static_images",
)


@dataclass
class ImageFile:
    """represents an image stored in the filesystem"""

    format_value: bytes
    filepath: str


@dataclass
class RawImage:
    """an image whose data was loaded into main-memory"""

    format_value: bytes
    data: bytes

    @staticmethod
    def from_display_image(display_image: DisplayImage):
        """turns a DisplayImage into a RawImage"""
        image_id = display_image.id.value
        match image_id:
            case ImageId.CUSTOM:
                return RawImage(
                    display_image.format.value, b"".join(display_image.data)
                )
            case ImageId.NONE:
                return None
            case _:
                image_file = IMAGE_ID_TO_STATIC_IMAGES.get(image_id)
                if image_file is None:
                    raise Exception(f"illegal image-id: '{image_id}'.")
                return RawImage.from_image_file(image_file)

    @staticmethod
    def from_image_file(image_file: ImageFile):
        """turns a ImageFile into a RawImage"""
        with open(image_file.filepath, "rb") as file:
            data = file.read()
        return RawImage(image_file.format_value, data)


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

    def _as_frames(
        self, data: bytes, width: int, height: int
    ) -> Iterable[AnimationFrame]:
        queue = Queue()
        Thread(
            target=self._load_frames_into_queue, args=(queue, data, width, height)
        ).start()
        while True:
            data, duration_ms = queue.get()
            if data is None:
                break
            yield AnimationFrame(duration_ms, PhotoImage(data=data))

    def _load_frames_into_queue(
        self, queue: Queue, data: bytes, width: int, height: int
    ) -> None:
        with PIL.Image.open(BytesIO(data)) as image:
            # iterate over frames of image
            for i in range(image.n_frames):
                # go to i-th frame of the image
                image.seek(i)
                # resize the current frame, to fit the screen-size
                resized = (
                    image.resize((width, height))
                    if image.width != width or image.height != height
                    else image
                )
                # buffer for storing binary data of image-frames
                data_buffer = BytesIO()
                # save the current frame in the data-buffer
                resized.save(data_buffer, "gif")
                # extract data from buffer and encode bytes as base64
                data = base64.b64encode(data_buffer.getvalue())
                # get the duration of the current frame
                duration_ms = image.info["duration"]
                # yield the extracted data
                queue.put((data, duration_ms))
            # 'None' -> all frames were processed
            queue.put((None, -1))


# maps an image-id to its corresponding image in the filesystem
IMAGE_ID_TO_STATIC_IMAGES: dict[int, ImageFile] = {
    ImageId.PIB_EYES_ANIMATED: ImageFile(
        ImageFormat.ANIMATED_GIF, STATIC_IMAGE_DIR + "/pib-eyes-animated.gif"
    ),
}

FORMAT_VALUE_TO_STR: dict[int, str] = {
    ImageFormat.ANIMATED_GIF: "gif",
    ImageFormat.PNG: "png",
    ImageFormat.JPEG: "jpeg",
}


class GuiApplication(Frame):

    def __init__(
        self,
        parent: Widget,
        image_queue: Queue[RawImage | None],
        inital_image: RawImage,
        *args,
        **kwargs,
    ):

        self._width = kwargs.setdefault("width", 100)
        self._height = kwargs.setdefault("height", 100)

        # call to the constructor of the superclass
        Frame.__init__(self, parent, *args, **kwargs)

        # store the image_queue to poll from it for images to show
        self.image_queue = image_queue

        # define a canvas where the main-image is displayed
        self.canvas = Canvas(
            self,
            width=self._width,
            height=self._height,
            borderwidth=0,
            highlightthickness=0,
        )
        self.canvas.place(x=0, y=0)

        # the current static-image/animation that is shown is stored here
        self.current_main_content: PhotoImage | Animation | None = None

        self._show_image(inital_image)

        # time until attempt to poll next image (in milliseconds)
        self.polling_timeout_ms = 10

        # intiate periodically polling for images in the queue
        self._poll_next_image()

        # position the widget in its parent
        self.grid()

    def _show_image(self, raw_image: RawImage) -> None:
        """update the main background image"""
        self.canvas.delete("all")
        if isinstance(self.current_main_content, Animation):
            self.current_main_content.stop()
        if raw_image.format_value == ImageFormat.ANIMATED_GIF:
            self._show_animated_gif(raw_image)
        else:
            self._show_static_image(raw_image)

    def _show_animated_gif(self, raw_image: RawImage) -> None:
        animation = Animation(raw_image.data, self._width, self._height)
        self.current_main_content = animation
        self._show_next_frame(animation)

    def _show_static_image(self, raw_image: RawImage) -> None:
        format_str = FORMAT_VALUE_TO_STR[raw_image.format_value]
        with PIL.Image.open(BytesIO(raw_image.data)) as image:
            resized = image.resize((self._width, self._height))
            # buffer for storing binary data of image
            data_buffer = BytesIO()
            # save the current frame in the data-buffer
            resized.save(data_buffer, format_str)
            data = base64.b64encode(data_buffer.getvalue())
        self.current_main_content = PhotoImage(data=data, format=format_str)
        self.canvas.create_image(0, 0, image=self.current_main_content, anchor="nw")

    def _show_next_frame(self, animation: Animation) -> None:
        try:
            frame: AnimationFrame = next(animation)
        except StopIteration:
            return
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=frame.photo_image, anchor="nw")
        self.canvas.after(frame.duration_ms, self._show_next_frame, animation)

    def _poll_next_image(self) -> None:
        if self.image_queue.qsize() != 0:
            image: Optional[RawImage] = self.image_queue.get()
            if image is None:
                self.winfo_toplevel().destroy()
                return
            else:
                # if an image is received, reset the timeout to the lowest
                # possible value
                self.polling_timeout_ms = 10
                self._show_image(image)
        else:
            # if no image, was received, increase the polling timeout
            # (value is capped at 160ms)
            self.polling_timeout_ms = min(2 * self.polling_timeout_ms, 160)
        self.after(self.polling_timeout_ms, self._poll_next_image)


class DisplayNode(Node):

    def __init__(self, image_queue: Queue[RawImage | None]) -> None:

        super().__init__("display")

        self.create_subscription(
            DisplayImage, "display_image", self.on_display_image_received, 1
        )

        self.image_queue = image_queue

        pib_eyes_animated = RawImage.from_image_file(
            IMAGE_ID_TO_STATIC_IMAGES[ImageId.PIB_EYES_ANIMATED]
        )
        self.image_queue.put(pib_eyes_animated)

        self.get_logger().info("Now Running DISPLAY")

    def on_display_image_received(self, display_image: DisplayImage):
        """callback function for the 'display_image'-topic subscriber"""
        try:
            raw_image = RawImage.from_display_image(display_image)
            self.image_queue.put(raw_image)
        except Exception as e:
            self.get_logger().error(f"error while showing image from topic: {e}.")


def run_gui_application(image_queue: Queue[RawImage | None]) -> None:
    while True:
        image = image_queue.get()
        if image is None:
            continue
        root = Tk()
        root.bind("<Escape>", lambda _: root.destroy())
        root.attributes("-fullscreen", True)
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        GuiApplication(root, image_queue, image, width=width, height=height)
        root.mainloop()


def run_display_node(image_queue: Queue[RawImage | None]) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    display_node = DisplayNode(image_queue)
    executor.add_node(display_node)
    executor.spin()
    display_node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    # the image-queue is used to send images from the ros-node to the
    # gui-application. The value is either a 'RawImage', which
    # the ros-node requests do be shown, or alternatively 'None', in
    # order to indicate that nothing should be shown (i.e. the gui-window
    # is closed)
    image_queue: Queue[RawImage | None] = Queue(maxsize=1)
    # run hui-application and ros in two separate threads
    Thread(daemon=True, target=run_display_node, args=(image_queue,)).start()
    run_gui_application(image_queue)


if __name__ == "__main__":
    main()
