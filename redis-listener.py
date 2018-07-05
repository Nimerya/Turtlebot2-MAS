import redis
    
from_linda = redis.Redis(host='127.0.0.1', port=6379)  # channel from linda
sub = from_linda.pubsub()
sub.subscribe('fromMAS')

print('listening for messages (subbed 1)...')
for item in sub.listen():
    print(item)
    if item['type'] == 'message':
        msg = item['data']
        print(msg)