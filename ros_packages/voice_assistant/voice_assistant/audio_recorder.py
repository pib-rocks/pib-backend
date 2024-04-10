import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from datatypes.action import RecordAudio

from threading import Lock

from public_api_client import public_voice_client
import wave

from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle



# these values define the pcm-encoding, in which the recorded
# audio will be received
BYTES_PER_SAMPLE = 2
FRAMES_PER_SECOND = 16000
NUM_CHANNELS = 1
CHUNKS_PER_SECOND = 10

FRAMES_PER_CHUNK = FRAMES_PER_SECOND // CHUNKS_PER_SECOND

# this value should not be hardcoded, as the optimal value 
# depends on the level of background noise. In the future,
# the user should have the opportunity to set this value
# (maybe it should also be persisted?), possibly via
# an 'adjust_to_background_noise' service
SILENCE_VOLUME_THRESHOLD = 0.1 * (BYTES_PER_SAMPLE * 256)



class AudioRecorderNode(Node):

    def __init__(self):

        super().__init__('audio_recorder')

        # used to determine, whether goals should be accepted or not
        self.current_goal: ServerGoalHandle | None = None
        self.current_goal_lock: Lock = Lock()
        self.accepted: bool = False

        # server for running local python-programs generated with blockly
        self.record_audio_server = ActionServer(
            self, 
            RecordAudio, 
            'record_audio', 
            execute_callback = self.record_audio,
            goal_callback = self.receive_goal,
            handle_accepted_callback=self.accept_goal,
            cancel_callback = (lambda _ : CancelResponse.ACCEPT),
            callback_group = MutuallyExclusiveCallbackGroup())

        self.get_logger().info('Now running AUDIO RECORDER')



    def receive_goal(self, _: ServerGoalHandle) -> GoalResponse:
        with self.current_goal_lock:
            if self.accepted:
                return GoalResponse.REJECT
            elif self.current_goal is not None and self.current_goal.is_active:
                return GoalResponse.REJECT
            else:
                self.accepted = True
                return GoalResponse. ACCEPT
            

            
    def accept_goal(self, goal_handle: ServerGoalHandle):
        with self.current_goal_lock:
            self.current_goal = goal_handle
            self.accepted = False
            self.current_goal.execute()


    
    def is_silent(self, data_chunk) -> bool:
        """Check whether a frame is below the minimum volume threshold"""
        as_ints = np.frombuffer(data_chunk, dtype=np.int16)
        m = np.abs(as_ints).mean()
        print(str(m) + " --- threshold: " + str(SILENCE_VOLUME_THRESHOLD)) # TODO: remove
        return np.abs(as_ints).mean() < SILENCE_VOLUME_THRESHOLD
    


    def record_audio(self, goal_handle: ServerGoalHandle):

        print("start recording")
        print(goal_handle.request)

        # a frame is considered siltent, if the mean amplitude across
        # all samples in that frame is below the threshold
        request: RecordAudio.Goal = goal_handle.request
        
        # these values indicate, after how many silent chunks, the recording is interrupted,
        # the 'before' value is used, if speech has not started yet
        # the 'after' value if used, after speech has already started
        # generally, the 'before' value should be greater than the 'afer' value
        max_silent_chunks_before = request.max_silent_seconds_before * CHUNKS_PER_SECOND
        max_silent_chunks_after = request.max_silent_seconds_after * CHUNKS_PER_SECOND

        # helpers for recording audio        
        chunks = []
        silent_chunks = 0
        max_silent_chunks = max_silent_chunks_before

        # create an pyaudio-input-stream for recording audio
        pya = pyaudio.PyAudio()
        stream = pya.open(
            format=pya.get_format_from_width(BYTES_PER_SAMPLE), 
            channels=NUM_CHANNELS, 
            rate=FRAMES_PER_SECOND, 
            input=True, 
            frames_per_buffer=FRAMES_PER_CHUNK)

        # record audio, until too many silent chunks were detected in a row
        # or if cancellation of the goal was requested
        while silent_chunks < max_silent_chunks:
            chunk = stream.read(FRAMES_PER_CHUNK, exception_on_overflow = False)
            chunks.append(chunk)
            if goal_handle.is_cancel_requested: 
                break
            if self.is_silent(chunk):
                silent_chunks += 1
            else: 
                max_silent_chunks = max_silent_chunks_after
                silent_chunks = 0

        # stop recording
        stream.stop_stream()
        stream.close()
        pya.terminate()

        # if cancel is requested, stop execution here
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return RecordAudio.Result()
        
        # if not canceled, transcribe recorded audio to text (via public-api)
        # and return the transcription
        # from now on, cancel requests are ignored (maybe change this in future?)
        
        # notifiy client, that the recording-phase has ended
        goal_handle.publish_feedback(RecordAudio.Feedback())

        # collect binary data
        data = b''.join(chunks)

        # TODO: public-api should accept raw pcm data in future -> this step unnecessary
        with wave.open('/home/pib/ros_working_dir/src/voice_assistant/pib_voice/pib_voice/tmp.wav', 'wb') as wave_file:
            wave_file.setnchannels(NUM_CHANNELS)
            wave_file.setsampwidth(BYTES_PER_SAMPLE)
            wave_file.setframerate(FRAMES_PER_SECOND)
            wave_file.writeframes(data)
            wave_file.close()
        with open('/home/pib/ros_working_dir/src/voice_assistant/pib_voice/pib_voice/tmp.wav', 'rb') as f: data = f.read()

        # transcribe the audio data
        text = public_voice_client.speech_to_text(data)

        # create an result object and return
        result = RecordAudio.Result()
        result.transcribed_text = text
        goal_handle.succeed()
        return result



def main(args=None):

    rclpy.init()
    node = AudioRecorderNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
