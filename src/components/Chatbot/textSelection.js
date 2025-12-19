/**
 * Text Selection Utility
 * Handles detection and extraction of selected text on the page
 */

/**
 * Get the currently selected text on the page
 * @returns {string} The selected text, or empty string if no text is selected
 */
export const getSelectedText = () => {
  // Use the standard window.getSelection() API
  const selection = window.getSelection();

  if (selection && selection.toString().trim() !== '') {
    return selection.toString().trim();
  }

  return '';
};

/**
 * Check if any text is currently selected on the page
 * @returns {boolean} True if text is selected, false otherwise
 */
export const isTextSelected = () => {
  return getSelectedText() !== '';
};

/**
 * Get the selected text range (for more advanced selection handling)
 * @returns {Range|null} The selected range object or null if no selection
 */
export const getSelectedRange = () => {
  const selection = window.getSelection();

  if (selection.rangeCount > 0) {
    return selection.getRangeAt(0);
  }

  return null;
};

/**
 * Clear the current text selection
 */
export const clearSelection = () => {
  const selection = window.getSelection();
  if (selection) {
    selection.removeAllRanges();
  }
};

/**
 * Get detailed information about the current text selection
 * @returns {Object} Selection details including text, start/end containers, etc.
 */
export const getSelectionDetails = () => {
  const selection = window.getSelection();

  if (selection && selection.toString().trim() !== '') {
    return {
      text: selection.toString().trim(),
      rangeCount: selection.rangeCount,
      anchorOffset: selection.anchorOffset,
      focusOffset: selection.focusOffset,
      isBackwards: selection.anchorOffset > selection.focusOffset
    };
  }

  return null;
};