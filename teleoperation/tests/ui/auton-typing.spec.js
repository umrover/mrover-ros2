const { test, expect } = require('@playwright/test');

test('typing input visible on ESTask', async ({ page }) => {
  await page.goto('/ESTask');
  await expect(page.getByTestId('pw-typing-input')).toBeVisible();
});

test('submit disabled for <3 chars', async ({ page }) => {
  await page.goto('/ESTask');

  const input = page.getByTestId('pw-typing-input');
  await expect(input).toBeVisible();
  await input.fill('AB');

  const submitBtn = page.getByTestId('pw-typing-submit');
  await expect(submitBtn).toBeDisabled();
});

test('submit enabled for 3+ chars', async ({ page }) => {
  await page.goto('/ESTask');

  const input = page.getByTestId('pw-typing-input');
  await expect(input).toBeVisible();
  await input.fill('ABC');

  const submitBtn = page.getByTestId('pw-typing-submit');
  await expect(submitBtn).toBeEnabled();
});

test('feedback table has 6 cells', async ({ page }) => {
  await page.goto('/ESTask');

  const table = page.getByTestId('pw-typing-feedback');
  await expect(table).toBeVisible();
  const cells = table.locator('td');
  await expect(cells).toHaveCount(6);
});
