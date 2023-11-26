import requests
import json
import gradio as gr


url = 'http://localhost:11434/api/generate'

headers = {
    'Content-Type': 'application/json',
}

history_chat= []

def generate_response(prompt):
    history_chat.append(prompt)

    full_prompt = '\n'.join(history_chat)

    data = {
        'model': 'safety',
        'stream': False,
        'prompt': full_prompt,
    }


    response = requests.post(url, headers=headers, data=json.dumps(data))

    if response.status_code == 200:
        response_text = response.text
        data = json.loads(response_text)
        actual_response = data['response']
        return actual_response
        history_chat.append(actual_response)
    else:
        print('Error:', response.status_code, response.text)
        return None

iface = gr.Interface(
    fn=generate_response,
    inputs=gr.Textbox(lines=2, placeholder='Enter your prompt here'),
    outputs='text'
)

iface.launch(share=True)