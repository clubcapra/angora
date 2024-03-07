from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from angora import Angora
import uvicorn

app = FastAPI()

class ChatRequest(BaseModel):
    message: str
    history: list
    model: str

@app.post("/chat/")
async def chat(chat_request: ChatRequest):
    try:
        print(chat_request.message, chat_request.history, chat_request.model)
        if chat_request.model == "local-small":
            angora_instance = Angora("tog/TinyLlama-1.1B-alpaca-chat-v1.5") 
        elif chat_request.model == "local-medium":
            angora_instance = Angora("teknium/OpenHermes-2.5-Mistral-7B") 
        elif chat_request.model == "capra-large":
            angora_instance = Angora("mistralai/Mixtral-8x7B-Instruct-v0.1") 
        else:
            angora_instance = Angora(model_name="microsoft/DialoGPT-small") 


        response_generator = angora_instance.predict(chat_request.message, chat_request.history)
        responses = []
        for response in response_generator:
            responses.append(response)
        
        if not responses:
            return {"response": "No response generated."}
        return {"response": response}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
