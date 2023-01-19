from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import numpy as np
import time
from PIL import Image
import os

import ctypes
import pygame


class VideoRecorderRuntime:
    """ 
    Class to record color and depth images from the Kinect v2.
    Displays the depth image in a pygame window.
    Color and depth images are saved to a directory with the current timestamp.
    The color image is registered to the depth image by the Kinects own algorithm. TODO?
    The images are currently not matched since the depth image recording seems to be much faster than the color image recording.
    Hence, every time a color image is recorded, the most recent depth image is assumed to be the matching one.
    """

    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Loop until the user clicks the close button.
        self._done = False

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)

        # width and height for the individual frames
        self.w_d = self._kinect.depth_frame_desc.Width
        self.h_d = self._kinect.depth_frame_desc.Height
        print(f"{self.w_d=} {self.h_d=}")
        self.w_rgb = self._kinect.color_frame_desc.Width
        self.h_rgb = self._kinect.color_frame_desc.Height
        print(f"{self.w_rgb=} {self.h_rgb=}")

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self.w_d, self.h_d),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Video Recorder")
        # back buffer surface for getting Kinect depth frames, 24 bit color, width and height equal to the Kinect depth frame size
        self._surface_d = pygame.Surface((self.w_d, self.h_d), 0, 24)
        self.rgb_list = []
        self.depth_list = []
        self.time_list = []
        self.setup_dir()

    def setup_dir(self):
        """ Create a directory for the current recording session. """
        ts_now = time.strftime("%Y%m%d-%H%M%S")
        self.dir_name = f"KinectData/{ts_now}"
        self.dir_name_rgb = os.path.join(self.dir_name, "rgb")
        self.dir_name_depth = os.path.join(self.dir_name, "depth")
        if not os.path.exists(self.dir_name_rgb):
            os.makedirs(self.dir_name_rgb)
        if not os.path.exists(self.dir_name_depth):
            os.makedirs(self.dir_name_depth)
        self.frame_count = 0

    def save_images(self, img_rgb, img_depth, ts):
        """ Save color and depth images to the current directory. """
        self.frame_count += 1
        # plt.imsave(f"{self.dir_name_rgb}/{self.frame_count:05d}.png", img_rgb)
        img_rgb = Image.fromarray(img_rgb, 'RGB')
        img_rgb.save(f"{self.dir_name_rgb}/{ts}.png")
        # save as 16 bit png
        # TUM dataset scales by 5 as well
        img_depth = Image.fromarray(img_depth * 5, 'I;16')
        img_depth.save(f"{self.dir_name_depth}/{ts}.png")
    
    def add_images(self, img_rgb, img_depth):
        self.rgb_list.append(img_rgb)
        self.depth_list.append(img_depth)
        self.time_list.append(time.time())

    def save_all_images(self):
        for i in range(len(self.rgb_list)):
            self.save_images(self.rgb_list[i], self.depth_list[i], self.time_list[i])
        print("Saved all images")

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def draw_depth_frame(self, frame, target_surface):
        target_surface.lock()
        f8 = np.uint8(frame.clip(1, 4000)/16.)
        frame8bit = np.dstack((f8, f8, f8))
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame8bit.ctypes.data, frame8bit.size)
        del address
        target_surface.unlock()

    def get_registered_image_naive(self, depth_to_color, color_frame):
        """
        DO NOT USE; this is a slow implementation.
        Get a registered image from a depth to color mapping and a color frame.
        This means an array with the same size as the depth frame, where each pixel is colored according to the color frame.
        """
        img_out = np.zeros((self.h_d, self.w_d, 3), dtype=np.uint8)
        for n in range(self.w_d * self.h_d):
            row, col = n // self.w_d, n % self.w_d
            p = depth_to_color[n]
            if np.isneginf(p.x) or np.isneginf(p.y):
                continue
            if p.x < 0 or p.y < 0:
                continue
            x, y = int(p.x), int(p.y)
            if x >= self.w_rgb or y >= self.h_rgb:
                continue
            img_out[row, col] = color_frame[y, x]
        return img_out

    def get_registered_image(self, depth_to_color, color_frame):
        """
        Get a registered image from a depth to color mapping and a color frame.
        This means an array with the same size as the depth frame, where each pixel is colored according to the color frame.
        TODO consider moving this to PyKindectRuntime instead
        """
        img_out = np.zeros((self.h_d, self.w_d, 3), dtype=np.uint8)
        c_coords = np.zeros((self.h_d, self.w_d, 2), dtype=np.float32)

        # move the data from the ctypes structure to a numpy array
        ctypes.memmove(c_coords.ctypes.data, depth_to_color, c_coords.nbytes)

        # clip to color frame size - this also removes negative infinity values
        c_coords[:, :, 0] = np.clip(c_coords[:, :, 0], 0, self.w_rgb - 1)
        c_coords[:, :, 1] = np.clip(c_coords[:, :, 1], 0, self.h_rgb - 1)
        c_coords = c_coords.astype(int)
        img_out = color_frame[c_coords[:, :, 1], c_coords[:, :, 0]]
        return img_out

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Getting frames and drawing
            if self._kinect.has_new_color_frame():
                # TODO this asssumes RGB is much faster than depth and we dont need to match frames - is this true?
                color_frame = self._kinect.get_last_color_frame()
                # reshape, convert to RGB, and remove alpha channel
                color_frame = color_frame.reshape(
                    (self.h_rgb, self.w_rgb, 4))[:, :, :3]
                # rgb to bgr
                color_frame = color_frame[:, :, ::-1]

                # get the latest depth frame and convert it to a 2D array
                depth_frame = self._kinect.get_last_depth_frame()
                depth_frame = depth_frame.reshape((self.h_d, self.w_d))
                # get the depth to color mapping and use it to get a registered image
                depth_to_color = self._kinect.get_depth_to_color_map()
                reg_img = self.get_registered_image(
                    depth_to_color, color_frame)
                # kinect is mirrored, so flip the image
                depth_frame = depth_frame[:, ::-1]
                reg_img = reg_img[:, ::-1]

                # self.save_images(reg_img, depth_frame)
                self.add_images(reg_img, depth_frame)
                time.sleep(0.1)

            if self._kinect.has_new_depth_frame():
                frame = self._kinect.get_last_depth_frame()
                self.draw_depth_frame(frame, self._surface_d)
                frame = None

            self._screen.blit(self._surface_d, (0, 0))
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self.save_all_images()
        self._kinect.close()
        pygame.quit()


if __name__ == "__main__":
    game = VideoRecorderRuntime()
    game.run()
