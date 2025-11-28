import chromadb

client = chromadb.PersistentClient(path="data/chroma")
collection = client.get_collection("task_embeddings")

# See all stored documents
results = collection.get()
print(f"Total documents: {len(results['ids'])}")
for i, (id, doc, meta) in enumerate(zip(results['ids'], results['documents'], results['metadatas'])):
    print(f"\n--- {id} ---")
    print(f"Task: {doc[:100]}...")
    print(f"Metadata: {meta}")