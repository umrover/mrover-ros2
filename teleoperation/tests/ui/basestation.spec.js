const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('basestation frontend loads', async ({ page }) => {
  await page.goto('http://localhost:8080/');
  await expect(page).toHaveTitle(/MRover/);
});

test('autonomy waypoint with string name persists after refresh', async ({ page }) => {
  const waypointName = `test-waypoint-${Date.now()}`;

  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');

  const addFromMapButton = page.getByTestId('pw-add-from-map');
  await expect(addFromMapButton).toBeVisible({ timeout: 15000 });
  await addFromMapButton.click();

  const modal = page.getByTestId('pw-waypoint-modal');
  await expect(modal).toHaveClass(/show/, { timeout: 5000 });

  const nameInput = page.getByTestId('pw-waypoint-name-input');
  await expect(nameInput).toBeVisible();
  await nameInput.clear();
  await nameInput.fill(waypointName);

  const addButton = page.getByTestId('pw-add-waypoint-submit');
  const savePromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'POST'
  );

  await addButton.click();
  await savePromise;

  await expect(modal).not.toHaveClass(/show/, { timeout: 5000 });

  const waypointStoreList = page.getByTestId('pw-waypoint-store-list');
  const waypointBeforeRefresh = waypointStoreList.getByTestId('pw-waypoint-name').filter({ hasText: waypointName });
  await expect(waypointBeforeRefresh).toBeVisible({ timeout: 5000 });

  const waypointsLoadPromise = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'GET',
    { timeout: 15000 }
  );

  await page.reload();
  await waypointsLoadPromise;
  await page.waitForLoadState('networkidle');

  await expect(page.getByTestId('pw-waypoint-store-list')).toBeVisible({ timeout: 15000 });

  const waypointAfterRefresh = page.getByTestId('pw-waypoint-store-list')
    .getByTestId('pw-waypoint-name')
    .filter({ hasText: waypointName });
  await expect(waypointAfterRefresh).toBeVisible({ timeout: 10000 });

  const waypointItem = page.getByTestId('pw-waypoint-store-item').filter({ hasText: waypointName });
  const deleteButton = waypointItem.locator('button:has-text("Delete")');

  if (await deleteButton.isVisible() && await deleteButton.isEnabled()) {
    const deletePromise = page.waitForResponse(
      (resp) => resp.url().includes('/api/waypoints/auton') && resp.request().method() === 'DELETE'
    );
    await deleteButton.click();
    await deletePromise.catch(() => {});
  }
});

test('autonomy navstate status color', async ({ page }) => {
  await page.goto('http://localhost:8080/AutonTask');
  await page.waitForLoadState('networkidle');
  await page.waitForTimeout(1000);

  const navStateValue = page.locator('.nav-state-value');
  await expect(navStateValue).toBeVisible({ timeout: 10000 });

  const navStatePanel = page.locator('.nav-state-panel');
  await expect(navStatePanel).toHaveClass(/nav-state--error/, { timeout: 5000 });

  await expect(navStateValue).toHaveText('OffState');
});
