import rclpy
from rclpy.node import Node
import gradio as gr
from .style import CapraStyle
import requests

class GradioROS2Node(Node):
    def __init__(self):
        super().__init__('gradio_ros2_node')
        self.capra_style = CapraStyle()

        self.launch_gradio_interface()

    def launch_gradio_interface(self):
        def llm_prediction_api(message, history):
            url = "http://localhost:8000/chat/"
            data = {"message": message, "history": history}
            response = requests.post(url, json=data)
            if response.status_code == 200:
                return response.json()["response"]
            else:
                return "Error: Could not communicate with the API service."

        send_btn = gr.Button("Send 👌", variant="primary")
        chatbot = gr.Chatbot(show_label=False)

        with gr.Blocks(title='ANGORA', theme=self.capra_style, css="footer{display:none !important}") as web_interface:
            header = gr.Label("A.N.G.O.R.A.: Automated Navigation and General Operations via ROS2 Assistant", show_label=False, color='primary')

            with gr.Row():
                authority_dropdown = gr.Dropdown(["ROS only", "Read Only", "Full authority"], type="index", value="Full authority", label="Permission level")
                database_checkbox = gr.Checkbox(label="Database (knowledge)", value=True)
                record_button = gr.Button("Record", variant="primary")
                stop_recording = gr.Button("Stop", variant="danger")
                input_audio = gr.Audio(label="Auto listening", sources=["microphone"], streaming=True)

            with gr.Row():
                label = gr.Textbox(label="Transcription", lines=10)
                with gr.Blocks():
                    chat_interface = gr.ChatInterface(llm_prediction_api, submit_btn=send_btn, chatbot=chatbot)
        
        web_interface.launch(allowed_paths=["assets"], share=False)

def main(args=None):
    rclpy.init(args=args)
    gradio_ros2_node = GradioROS2Node()

    try:
        rclpy.spin(gradio_ros2_node)
    except KeyboardInterrupt:
        gradio_ros2_node.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        gradio_ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
