from transformers import AutoModelForCausalLM, AutoTokenizer, StoppingCriteriaList, TextIteratorStreamer, pipeline, StoppingCriteria
import torch
from threading import Thread

class StopOnTokens(StoppingCriteria):
    def __init__(self, stop_ids=None):
        self.stop_ids = stop_ids or [29, 0]

    def __call__(self, input_ids: torch.LongTensor, scores: torch.FloatTensor, **kwargs) -> bool:
        return input_ids[0][-1] in self.stop_ids


class Angora():
    def __init__(self, model_name="tog/TinyLlama-1.1B-alpaca-chat-v1.5", gpu=False, mode="base"):
        self.gpu = gpu
        self.model_name = model_name
        self.tokenizer, self.model = self.setup_model_and_embeddings()
        
    def setup_model_and_embeddings(self): 
        base_model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
        )
        model = base_model
        tokenizer = AutoTokenizer.from_pretrained(self.model_name)

        return tokenizer, model
    
    def predict(self, message, history):
        history_transformer_format = history + [[message, ""]]
        stop = StopOnTokens()

        messages = "".join(["".join(["\n<human>:"+item[0], "\n<bot>:"+item[1]])  # curr_system_message +
                            for item in history_transformer_format])
        
        streamer = TextIteratorStreamer(
            self.tokenizer, timeout=10., skip_prompt=True, skip_special_tokens=True)

        standard_pipeline = pipeline(
            "text-generation",
            return_full_text=True,  # langchain expects the full text
            model=self.model,
            tokenizer=self.tokenizer,
            max_new_tokens=300,
            pad_token_id=self.tokenizer.eos_token_id,
            repetition_penalty=1.2,
            streamer=streamer,
            stopping_criteria=StoppingCriteriaList([stop])
        )
        t = Thread(target=standard_pipeline, args=(messages,))

        t.start()
        
        partial_message = ""

        for new_token in streamer:
            if new_token != '<':
                partial_message += new_token
                yield partial_message
