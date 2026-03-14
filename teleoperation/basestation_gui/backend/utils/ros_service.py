import asyncio
import logging

logger = logging.getLogger(__name__)


async def call_service_async(client, request, timeout=10.0):
    if not client.service_is_ready():
        logger.warning(f"Service {client.srv_name} not ready, skipping call")
        return None

    loop = asyncio.get_running_loop()
    future = client.call_async(request)
    event = asyncio.Event()
    result = None

    def on_done(fut):
        nonlocal result
        try:
            result = fut.result()
        except Exception as exc:
            logger.error(f"Service call failed: {exc}")
        loop.call_soon_threadsafe(event.set)

    future.add_done_callback(on_done)

    try:
        await asyncio.wait_for(event.wait(), timeout=timeout)
        return result
    except asyncio.TimeoutError:
        future.cancel()
        return None
