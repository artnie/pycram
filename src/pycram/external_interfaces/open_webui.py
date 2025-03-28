import json

import requests


local_token = 'sk-a62255e7907040058b467e24dc0f062f'


def get_models(token=local_token):
    url = 'http://localhost:3000/api/models'
    headers = {
        'Authorization': f'Bearer {token}'
    }
    response = requests.get(url, headers=headers)
    return response.json()


def chat_with_model(token=local_token, model='llama3:latest'):
    url = 'http://localhost:3000/api/chat/completions'
    headers = {
        'Authorization': f'Bearer {token}',
        'Content-Type': 'application/json'
    }
    data = {
        'model': f'{model}',
        'messages': [
            {
                'role': 'user',
                'content': 'Why is the sky blue?'
            }
        ]
    }
    response = requests.post(url, headers=headers, data=json.dumps(data))
    return response.json()


def pull_model(model):
    url = 'http://localhost:11434/api/pull'
    headers = {
        'Content-Type': 'application/json'
    }
    data = {
        'model': f'{model}',
    }
    response = requests.post(url, headers=headers, data=json.dumps(data))
    return response.json()


def chat_with_collection(token, model, query, collection_id):
    url = 'http://localhost:3000/api/chat/completions'
    headers = {
        'Authorization': f'Bearer {token}',
        'Content-Type': 'application/json'
    }
    payload = {
        'model': model,
        'messages': [{'role': 'user',
                      'content': query}],
        'files': [{'type': 'collection', 'id': collection_id}]
    }
    response = requests.post(url, headers=headers, json=payload)
    return response.json()