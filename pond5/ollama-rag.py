from langchain.llms import Ollama
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableLambda, RunnablePassthrough
from langchain.document_loaders import TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma
import gradio as gr

# load the document and split it into chunks
loader = TextLoader("./workshop_rules.txt")
documents = loader.load()

# split it into chunks
text_splitter = CharacterTextSplitter(chunk_size=1200, chunk_overlap=0)
docs = text_splitter.split_documents(documents)

# create the open-source embedding function
embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")

# load it into Chroma
vectorstore = Chroma.from_documents(docs, embedding_function)

retriever = vectorstore.as_retriever()

template = """Answer the question based only on the following context:
{context}

Question: {question}
"""
prompt = ChatPromptTemplate.from_template(template)

model = Ollama(model="safety")

chain = (
    {"context": retriever, "question": RunnablePassthrough()}
    | prompt
    | model
)


def chatbot(prompt):
    response_complete = []
    response = ''

    for s in chain.stream(prompt):
        response += s
        if '<' in response:
            response = response.removesuffix('<')
            break
        yield response
        
    response_complete.append((prompt, response))
    return '', response_complete
        
    

demo = gr.Interface(
    fn=chatbot, 
    inputs=gr.Textbox(lines=2, placeholder='Escreva sua dúvida aqui'), 
    outputs='text'
).queue()

if __name__ == "__main__":
    demo.launch(show_api=True) 

