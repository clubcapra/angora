from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from angora import Angora  # Ensure this imports your modified Angora class
import uvicorn

app = FastAPI()

class ChatRequest(BaseModel):
    message: str
    history: list

@app.post("/chat/")
async def chat(chat_request: ChatRequest):
    try:
        angora_instance = Angora(model_name="microsoft/DialoGPT-small")  # Consider passing parameters or initializing this elsewhere
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
