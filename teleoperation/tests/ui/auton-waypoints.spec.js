const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('default waypoints load', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  const storeItems = page.getByTestId('pw-waypoint-store-item');
  await expect(storeItems.first()).toBeVisible({ timeout: 15000 });
  const count = await storeItems.count();
  expect(count).toBeGreaterThanOrEqual(8);
});

test('add from map opens modal', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  const addBtn = page.getByTestId('pw-add-from-map');
  await expect(addBtn).toBeVisible({ timeout: 15000 });
  await addBtn.click();
  const modal = page.getByTestId('pw-waypoint-modal');
  await expect(modal).toHaveClass(/show/, { timeout: 5000 });
});

test('add waypoint with name', async ({ page }) => {
  const wpName = `test-wp-${Date.now()}`;
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toHaveClass(/show/, { timeout: 5000 });

  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);

  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const storeList = page.getByTestId('pw-waypoint-store-list');
  const wp = storeList.getByTestId('pw-waypoint-name').filter({ hasText: wpName });
  await expect(wp).toBeVisible({ timeout: 5000 });

  // Cleanup
  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  const deleteBtn = item.locator('button:has-text("Delete")');
  if (await deleteBtn.isVisible()) {
    await deleteBtn.click();
  }
});

test('waypoint name persists after refresh', async ({ page }) => {
  const wpName = `persist-${Date.now()}`;
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toHaveClass(/show/, { timeout: 5000 });
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);

  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  await page.reload();
  await page.waitForLoadState('networkidle');

  const wp = page.getByTestId('pw-waypoint-store-list')
    .getByTestId('pw-waypoint-name')
    .filter({ hasText: wpName });
  await expect(wp).toBeVisible({ timeout: 10000 });

  // Cleanup
  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  const deleteBtn = item.locator('button:has-text("Delete")');
  if (await deleteBtn.isVisible() && await deleteBtn.isEnabled()) {
    const delPromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'DELETE'
    );
    await deleteBtn.click();
    await delPromise.catch(() => {});
  }
});

test('default waypoints cannot be deleted', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const firstDefault = page.getByTestId('pw-waypoint-store-item').filter({ hasText: 'No Search 1' });
  await expect(firstDefault).toBeVisible({ timeout: 15000 });

  const deleteBtn = firstDefault.locator('button:has-text("Delete")');
  const isVisible = await deleteBtn.isVisible().catch(() => false);
  if (isVisible) {
    await expect(deleteBtn).toBeDisabled();
  }
});

test('user waypoint can be deleted', async ({ page }) => {
  const wpName = `del-test-${Date.now()}`;
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toHaveClass(/show/, { timeout: 5000 });
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);
  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const item = page.getByTestId('pw-waypoint-store-item').filter({ hasText: wpName });
  await expect(item).toBeVisible({ timeout: 5000 });

  const deleteBtn = item.locator('button:has(.bi-trash-fill)');
  await expect(deleteBtn).toBeVisible();
  await expect(deleteBtn).toBeEnabled();
  await deleteBtn.click();

  await expect(item).not.toBeVisible({ timeout: 10000 });
});

test('add waypoint to route', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const firstItem = page.getByTestId('pw-waypoint-store-item').first();
  await expect(firstItem).toBeVisible({ timeout: 15000 });

  const addBtn = firstItem.locator('button:has(.bi-plus-lg)');
  await addBtn.click();

  const routeItems = page.getByTestId('pw-route-item');
  await expect(routeItems.first()).toBeVisible({ timeout: 5000 });
});

test('delete from route', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const firstItem = page.getByTestId('pw-waypoint-store-item').first();
  await expect(firstItem).toBeVisible({ timeout: 15000 });

  const routeItems = page.getByTestId('pw-route-item');
  const initialCount = await routeItems.count();

  const addBtn = firstItem.locator('button:has(.bi-plus-lg)');
  await addBtn.click();

  await expect(routeItems).toHaveCount(initialCount + 1, { timeout: 5000 });

  const lastRouteItem = routeItems.last();
  const deleteBtn = lastRouteItem.locator('button:has(.bi-trash-fill)');
  await deleteBtn.click();

  await expect(routeItems).toHaveCount(initialCount, { timeout: 5000 });
});

test('all costmaps toggle', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const toggleBtn = page.getByTestId('pw-costmap-toggle-all');
  await expect(toggleBtn).toBeVisible({ timeout: 15000 });

  const initialClass = await toggleBtn.getAttribute('class');
  await toggleBtn.click();
  await page.waitForTimeout(300);
  const newClass = await toggleBtn.getAttribute('class');
  expect(initialClass).not.toBe(newClass);
});

test('reset clears user waypoints', async ({ page }) => {
  const wpName = `reset-test-${Date.now()}`;
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  await page.getByTestId('pw-add-from-map').click();
  await expect(page.getByTestId('pw-waypoint-modal')).toHaveClass(/show/, { timeout: 5000 });
  await page.getByTestId('pw-waypoint-name-input').clear();
  await page.getByTestId('pw-waypoint-name-input').fill(wpName);
  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );
  await page.getByTestId('pw-add-waypoint-submit').click();
  await savePromise;

  const resetBtn = page.getByTestId('pw-reset-waypoints-btn');
  await resetBtn.click();

  const confirmBtn = page.locator('.modal.show .btn-danger:has-text("Reset")');
  await expect(confirmBtn).toBeVisible({ timeout: 5000 });

  const clearPromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton/clear') && resp.request().method() === 'DELETE'
  );
  await confirmBtn.click();
  await clearPromise;

  await page.waitForTimeout(1000);
  const wpAfter = page.getByTestId('pw-waypoint-store-list')
    .getByTestId('pw-waypoint-name')
    .filter({ hasText: wpName });
  await expect(wpAfter).not.toBeVisible({ timeout: 5000 });
});
