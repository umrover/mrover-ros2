import asyncio


async def call_service_async(client, request, timeout=1.0):
    loop = asyncio.get_running_loop()
    future = client.call_async(request)
    event = asyncio.Event()
    result_holder = [None]

    def on_done(fut):
        result_holder[0] = fut.result()
        loop.call_soon_threadsafe(event.set)

    future.add_done_callback(on_done)

    try:
        await asyncio.wait_for(event.wait(), timeout=timeout)
        return result_holder[0]
    except asyncio.TimeoutError:
        future.cancel()
        return None
