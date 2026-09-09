from ollama import chat

response = chat(
    model='llama3.2',
    messages=[
        {
            'role': 'user',
            'content': 'What sensors does a differential drive robot typically use?'
        }
    ]
)

print(response.message.content)
