import asyncio
import logging

logger = logging.getLogger(__name__)


async def call_service_async(client, request, timeout=10.0) -> tuple:
    if not client.service_is_ready():
        reason = f"Service {client.srv_name} not ready"
        logger.warning(reason)
        return None, reason

    logger.info(f"Service {client.srv_name} is ready, sending request")

    loop = asyncio.get_running_loop()
    future = client.call_async(request)
    event = asyncio.Event()
    result = None
    error = None

    def on_done(fut):
        nonlocal result, error
        try:
            result = fut.result()
            logger.info(f"Service {client.srv_name} call completed successfully")
        except Exception as exc:
            error = f"Service call to {client.srv_name} failed: {exc}"
            logger.error(error)
        loop.call_soon_threadsafe(event.set)

    future.add_done_callback(on_done)

    try:
        await asyncio.wait_for(event.wait(), timeout=timeout)
        return result, error
    except asyncio.TimeoutError:
        logger.error(f"Service {client.srv_name} call timed out after {timeout}s")
        future.cancel()
        return None, f"Service {client.srv_name} timed out after {timeout}s"
