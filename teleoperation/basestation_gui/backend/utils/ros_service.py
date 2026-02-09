import asyncio


async def call_service_async(client, request, timeout=10.0):
    loop = asyncio.get_running_loop()
    service_ready = await loop.run_in_executor(
        None, lambda: client.wait_for_service(timeout_sec=1.0)
    )
    if not service_ready:
        return None

    future = client.call_async(request)
    try:
        await asyncio.wait_for(
            loop.run_in_executor(None, future.result),
            timeout=timeout
        )
        return future.result()
    except asyncio.TimeoutError:
        return None
