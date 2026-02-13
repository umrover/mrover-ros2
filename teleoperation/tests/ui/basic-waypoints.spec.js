const { test, expect } = require('@playwright/test');

test('add waypoint form visible', async ({ page }) => {
  await page.goto('/DMTask');
  await expect(page.getByTestId('pw-basic-wp-name')).toBeVisible();
  await expect(page.getByTestId('pw-basic-wp-add-btn')).toBeVisible();
});

test('add waypoint creates entry', async ({ page }) => {
  await page.goto('/DMTask');

  const nameInput = page.getByTestId('pw-basic-wp-name');
  await expect(nameInput).toBeVisible();
  await nameInput.clear();
  await nameInput.fill('TestBasicWP');

  await page.getByTestId('pw-basic-wp-add-btn').click();

  const list = page.getByTestId('pw-basic-wp-list');
  await expect(list.locator(':text("TestBasicWP")')).toBeVisible();
});

test('clear empties list', async ({ page }) => {
  await page.goto('/DMTask');

  const nameInput = page.getByTestId('pw-basic-wp-name');
  await expect(nameInput).toBeVisible();
  await nameInput.clear();
  await nameInput.fill('ClearTest');
  await page.getByTestId('pw-basic-wp-add-btn').click();
  await page.waitForTimeout(500);

  await page.getByTestId('pw-basic-wp-clear-btn').click();

  const confirmBtn = page.locator('.cmd-modal-backdrop .cmd-btn-danger');
  await expect(confirmBtn).toBeVisible();

  const clearResponse = page.waitForResponse(
    (resp) => resp.url().includes('/api/waypoints/basic/clear') && resp.status() === 200
  );
  await confirmBtn.click();
  await clearResponse;

  await page.reload();
  const list = page.getByTestId('pw-basic-wp-list');
  const items = list.locator('.cmd-list-item');
  await expect(items).toHaveCount(0);
});

test('view recordings button visible', async ({ page }) => {
  await page.goto('/DMTask');
  await expect(page.getByTestId('pw-basic-wp-recordings-btn')).toBeVisible();
});

test('start recording button visible', async ({ page }) => {
  await page.goto('/DMTask');
  const recordBtn = page.locator('button:has-text("Start Recording")').first();
  await expect(recordBtn).toBeVisible();
});
