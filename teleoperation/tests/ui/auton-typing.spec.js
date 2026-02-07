const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('typing input visible on ESTask', async ({ page }) => {
  await page.goto('http://localhost:8080/ESTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-typing-input')).toBeVisible({ timeout: 15000 });
});

test('submit disabled for <3 chars', async ({ page }) => {
  await page.goto('http://localhost:8080/ESTask');
  await page.waitForLoadState('networkidle');

  const input = page.getByTestId('pw-typing-input');
  await expect(input).toBeVisible({ timeout: 15000 });
  await input.fill('AB');

  const submitBtn = page.getByTestId('pw-typing-submit');
  await expect(submitBtn).toBeDisabled();
});

test('submit enabled for 3+ chars', async ({ page }) => {
  await page.goto('http://localhost:8080/ESTask');
  await page.waitForLoadState('networkidle');

  const input = page.getByTestId('pw-typing-input');
  await expect(input).toBeVisible({ timeout: 15000 });
  await input.fill('ABC');

  const submitBtn = page.getByTestId('pw-typing-submit');
  await expect(submitBtn).toBeEnabled();
});

test('feedback table has 6 cells', async ({ page }) => {
  await page.goto('http://localhost:8080/ESTask');
  await page.waitForLoadState('networkidle');

  const table = page.getByTestId('pw-typing-feedback');
  await expect(table).toBeVisible({ timeout: 15000 });
  const cells = table.locator('td');
  await expect(cells).toHaveCount(6);
});
