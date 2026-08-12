const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('add waypoint form visible', async ({ page }) => {
  await page.goto('/DMTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-basic-wp-name')).toBeVisible({ timeout: 15000 });
  await expect(page.getByTestId('pw-basic-wp-add-btn')).toBeVisible();
});

// FIXME fails when there is already a waypoint
test('add waypoint creates entry', async ({ page }) => {
  await page.goto('/DMTask');
  await page.waitForLoadState('networkidle');

  const nameInput = page.getByTestId('pw-basic-wp-name');
  await expect(nameInput).toBeVisible({ timeout: 15000 });
  await nameInput.clear();
  await nameInput.fill('TestBasicWP');

  await page.getByTestId('pw-basic-wp-add-btn').click();

  const list = page.getByTestId('pw-basic-wp-list');
  await expect(list.locator(':text("TestBasicWP")')).toBeVisible({ timeout: 5000 });
});

test('clear empties list', async ({ page }) => {
  await page.goto('/DMTask');
  await page.waitForLoadState('networkidle');

  const nameInput = page.getByTestId('pw-basic-wp-name');
  await expect(nameInput).toBeVisible({ timeout: 15000 });
  await nameInput.clear();
  await nameInput.fill('ClearTest');
  await page.getByTestId('pw-basic-wp-add-btn').click();
  await page.waitForTimeout(500);

  await page.getByTestId('pw-basic-wp-clear-btn').click();

  const confirmBtn = page.locator('.modal-footer .btn-danger:has-text("Clear")');
  await expect(confirmBtn).toBeVisible({ timeout: 5000 });

  const clearResponse = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/basic/clear') && resp.status() === 200
  );
  await confirmBtn.click();
  await clearResponse;

  await page.reload();
  await page.waitForLoadState('networkidle');
  const list = page.getByTestId('pw-basic-wp-list');
  const items = list.locator('.cmd-list-item');
  await expect(items).toHaveCount(0, { timeout: 15000 });
});

test('view recordings button visible', async ({ page }) => {
  await page.goto('/DMTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-basic-wp-recordings-btn')).toBeVisible({ timeout: 15000 });
});

test('start recording buttons visible', async ({ page }) => {
  await page.goto('/DMTask');
  await page.waitForLoadState('networkidle');
  // const recordBtn = page.locator('button:has-text("Start Recording")').first();
  const recordBtns = page.getByTestId('pw-recording-btn-div');
  await expect(recordBtns).toBeVisible({ timeout: 15000 });

  const items = recordBtns.locator("button")
  await expect(items).toHaveCount(2, { timeout: 5000})
});
