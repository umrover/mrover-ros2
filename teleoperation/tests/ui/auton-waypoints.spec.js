const { test, expect } = require('@playwright/test');

test('default waypoints load', async ({ page }) => {
  await page.goto('/AutonTask');
  const storeItems = page.getByTestId('pw-waypoint-store-item');
  await expect(storeItems.first()).toBeVisible();
  const count = await storeItems.count();
  expect(count).toBeGreaterThanOrEqual(8);
});

test('add from map opens modal', async ({ page }) => {
  await page.goto('/AutonTask');
  const addBtn = page.getByTestId('pw-add-from-map');
  await expect(addBtn).toBeVisible();
  await addBtn.click();
  await expect(page.getByTestId('pw-waypoint-modal')).toBeVisible();
});

test('add waypoint with name', async ({ page }) => {
  const wpName = `test-wp-${Date.now()}`;
  await page.goto('/AutonTask');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toBeVisible();

  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);

  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const storeList = page.getByTestId('pw-waypoint-store-list');
  const wp = storeList.getByTestId('pw-waypoint-name').filter({ hasText: wpName });
  await expect(wp).toBeVisible();

  // Cleanup
  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  const deleteBtn = item.locator('button:has(.bi-trash-fill)');
  if (await deleteBtn.isVisible()) {
    await deleteBtn.click();
  }
});

test('waypoint name persists after refresh', async ({ page }) => {
  const wpName = `persist-${Date.now()}`;
  await page.goto('/AutonTask');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toBeVisible();
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);

  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  await page.reload();

  const wp = page.getByTestId('pw-waypoint-store-list')
    .getByTestId('pw-waypoint-name')
    .filter({ hasText: wpName });
  await expect(wp).toBeVisible();

  // Cleanup
  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  const deleteBtn = item.locator('button:has(.bi-trash-fill)');
  if (await deleteBtn.isVisible() && await deleteBtn.isEnabled()) {
    const delPromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'DELETE'
    );
    await deleteBtn.click();
    await delPromise.catch(() => {});
  }
});

test('default waypoints cannot be deleted', async ({ page }) => {
  await page.goto('/AutonTask');

  const firstDefault = page.getByTestId('pw-waypoint-store-item').filter({ hasText: 'No Search 1' });
  await expect(firstDefault).toBeVisible();

  const deleteBtn = firstDefault.locator('button:has(.bi-trash-fill)');
  const isVisible = await deleteBtn.isVisible().catch(() => false);
  if (isVisible) {
    await expect(deleteBtn).toBeDisabled();
  }
});

test('user waypoint can be deleted', async ({ page }) => {
  const wpName = `del-test-${Date.now()}`;
  await page.goto('/AutonTask');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toBeVisible();
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);
  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  await expect(item).toBeVisible();

  const deleteBtn = item.locator('button:has(.bi-trash-fill)');
  await expect(deleteBtn).toBeVisible();
  await expect(deleteBtn).toBeEnabled();
  await deleteBtn.click();

  await expect(item).not.toBeVisible();
});

test('add waypoint to route', async ({ page }) => {
  await page.goto('/AutonTask');

  const firstItem = page.getByTestId('pw-waypoint-store-item').first();
  await expect(firstItem).toBeVisible();

  const addBtn = firstItem.locator('button:has(.bi-plus-lg)');
  await addBtn.click();

  const routeItems = page.getByTestId('pw-route-item');
  await expect(routeItems.first()).toBeVisible();
});

test('delete from route', async ({ page }) => {
  await page.goto('/AutonTask');

  const firstItem = page.getByTestId('pw-waypoint-store-item').first();
  await expect(firstItem).toBeVisible();

  const routeItems = page.getByTestId('pw-route-item');
  const initialCount = await routeItems.count();

  const addBtn = firstItem.locator('button:has(.bi-plus-lg)');
  await addBtn.click();

  await expect(routeItems).toHaveCount(initialCount + 1);

  const lastRouteItem = routeItems.last();
  const deleteBtn = lastRouteItem.locator('button:has(.bi-trash-fill)');
  await deleteBtn.click();

  await expect(routeItems).toHaveCount(initialCount);
});

test('all costmaps toggle', async ({ page }) => {
  await page.goto('/AutonTask');

  const toggleBtn = page.getByTestId('pw-costmap-toggle-all');
  await expect(toggleBtn).toBeVisible();

  const initialClass = await toggleBtn.getAttribute('class');
  await toggleBtn.click();
  await page.waitForTimeout(300);
  const newClass = await toggleBtn.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('reset clears user waypoints', async ({ page }) => {
  const wpName = `reset-test-${Date.now()}`;
  await page.goto('/AutonTask');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toBeVisible();
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);
  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const resetBtn = page.getByTestId('pw-reset-waypoints-btn');
  await resetBtn.click();

  const confirmBtn = page.locator('.cmd-modal-backdrop .cmd-btn-danger');
  await expect(confirmBtn).toBeVisible();

  const clearPromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton/rebuild') && resp.request().method() === 'POST'
  );
  await confirmBtn.click();
  await clearPromise;

  await page.waitForTimeout(1000);
  const wpAfter = page.getByTestId('pw-waypoint-store-list')
    .getByTestId('pw-waypoint-name')
    .filter({ hasText: wpName });
  await expect(wpAfter).not.toBeVisible();
});
